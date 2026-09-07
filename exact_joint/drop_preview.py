"""Visible analytic pre-impact preview in Kit; deliberately stops before impact."""

import asyncio
import json
import time
from datetime import datetime, timezone

import omni.kit.app
import omni.ui as ui
import omni.usd
from isaacsim.core.experimental.utils import app as app_utils
from isaacsim.core.rendering_manager import ViewportManager
from omni.kit.viewport.utility import get_active_viewport_window
from pxr import Gf, Usd, UsdGeom

from exact_joint.drop_test import DropCase
from exact_joint.live_display import LiveFemDisplay


class DropPreview:
    """Own only a root translation and preview UI; leave the FEM mesh unchanged."""

    def __init__(self, lab, case=None):
        self.lab = lab
        self.case = case or DropCase()
        self.stage = lab.scene.stage
        self.task = None
        self.running = False
        self.closed = False
        self.elapsed = 0.0
        self.frames = 0
        self.window = None
        self.hud = None
        self.root = self.stage.GetPrimAtPath("/World/ExactLeg")
        self.op = None
        self.suspended = False
        self.diagnostic = None

    async def start(self):
        if not self.lab.connected():
            raise ValueError("Reconnect the exact knee before opening the drop preview.")
        # Refuse unsupported user transforms instead of silently redefining height.
        cache = UsdGeom.XformCache()
        for prim in (self.root, self.stage.GetPrimAtPath("/World")):
            if cache.GetLocalToWorldTransform(prim) != Gf.Matrix4d(1):
                raise ValueError("Preview requires the original untransformed whole-leg root.")
        if self.root.GetAttribute("xformOp:translate:dropPreview"):
            raise ValueError("An existing drop-preview transform must be restored first.")
        for path in ("/World/ExactLeg/Foot", "/World/Plinth"):
            if not self.stage.GetPrimAtPath(path):
                raise ValueError("Missing drop-preview geometry: " + path)
        self.original_type = self.root.GetTypeName()
        self.original_order = self.root.GetAttribute("xformOpOrder").Get()
        self.original_order_authored = self.root.GetAttribute("xformOpOrder").HasAuthoredValueOpinion()
        if app_utils.is_playing():
            app_utils.stop(commit=False)
            await omni.kit.app.get_app().next_update_async()
        self.lab.neutral()
        self.lab.calculate()
        self.lab.paused = True
        self.suspended = True
        if self.lab.task:
            self.lab.task.cancel()
            await self.lab.task
        self.lab.window.visible = False
        self.lab.live_display.close()
        self.diagnostic = UsdGeom.Imageable(self.stage.GetPrimAtPath("/World/ExactFemDisplay"))
        self.diagnostic_visibility = self.diagnostic.GetVisibilityAttr().Get()
        self.diagnostic_active = self.diagnostic.GetPrim().IsActive()
        self.diagnostic.MakeInvisible()
        # Deactivate the display subtree as well: cached curve glyphs can linger
        # in RTX when only inherited visibility changes after their points clear.
        self.diagnostic.GetPrim().SetActive(False)
        self.lab.scene.update(self.lab.results, show_stress=False)
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        foot = bounds.ComputeWorldBound(self.stage.GetPrimAtPath("/World/ExactLeg/Foot")).ComputeAlignedRange()
        floor = bounds.ComputeWorldBound(self.stage.GetPrimAtPath("/World/Plinth")).ComputeAlignedRange()
        self.original_clearance = foot.GetMin()[2] - floor.GetMax()[2]
        self.start_offset = self.case.height_m - self.original_clearance
        self.root.SetTypeName("Xform")
        self.op = UsdGeom.Xformable(self.root).AddTranslateOp(opSuffix="dropPreview")
        self.build_ui()
        await omni.kit.app.get_app().next_update_async()
        target = next((window for window in ui.Workspace.get_windows() if window.title == "Stage"), None)
        if target:
            self.window.dock_in(target, ui.DockPosition.SAME)
        self.window.focus()
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[.43, -.61, .35], target=[0, 0, .12])
        self.set_time(0.0)
        self.task = asyncio.ensure_future(self.loop())

    def build_ui(self):
        self.window = ui.Window("70 mm drop - PRE-IMPACT ONLY", width=460, height=620)
        with self.window.frame, ui.ScrollingFrame():
            with ui.VStack(spacing=8, height=0):
                ui.Label("70 mm WHOLE-LEG RELEASE", height=34, style={"font_size": 20})
                ui.Label("PRE-IMPACT ONLY\nSURVIVAL NOT EVALUATED", height=66,
                         style={"font_size": 22, "color": 0xFF80DFFF})
                ui.Label("Original JSON knee only; rigid upper/lower links.\nUpright, foot first; rigid floor; cables unloaded.", height=46)
                ui.Label(f"ASSUMED mass: {self.case.mass_kg*1000:g} g (upper {self.case.upper_mass_kg*1000:g} g + lower {self.case.lower_mass_kg*1000:g} g).\nIncludes joint/plates; no payload. Not measured or optimized.", height=50, word_wrap=True)
                contact = self.case.sample(self.case.contact_time_s)
                ui.Label(f"{self.case.playback_speed:g}x playback; physical time shown below.\nFall time: {self.case.contact_time_s*1000:.2f} ms; contact speed: {contact['incident_downward_speed_m_s']:.3f} m/s.\nIncident energy: {contact['kinetic_energy_j']*1000:.2f} mJ, NOT a joint capacity.", height=70, word_wrap=True)
                self.telemetry = ui.Label("", height=80, word_wrap=True)
                with ui.HStack(height=32):
                    ui.Button("Replay 70 mm", clicked_fn=self.replay)
                    ui.Button("Pause / resume fall", clicked_fn=self.toggle_pause)
                ui.Button("Return to cable FEM", height=32, clicked_fn=lambda: asyncio.ensure_future(self.restore()))
                ui.Button("Save drop assumptions", height=30, clicked_fn=self.save)
                self.feedback = ui.Label("Ready. Click Replay 70 mm. Stops at first foot contact.", height=60, word_wrap=True)
                ui.Label("No impact stress or failure calculation is running.\nAn unchanged joint here does NOT mean it survived.\nMesh, contact, inertia and material/bond failure need validation.", height=75, word_wrap=True)
        self.hud = get_active_viewport_window().get_frame("exact_joint_drop_legend")
        with self.hud:
            with ui.VStack():
                ui.Spacer(height=64)
                with ui.HStack(height=92):
                    ui.Spacer(width=90)
                    with ui.ZStack():
                        ui.Rectangle(style={"background_color": 0xE9222428}, height=92)
                        self.legend = ui.Label("", word_wrap=True, style={"font_size": 19, "color": 0xFF80DFFF, "margin_width": 12})
                    ui.Spacer(width=25)
                ui.Spacer()

    def set_time(self, elapsed):
        sample = self.case.sample(elapsed)
        self.elapsed = sample["time_s"]
        self.op.Set(Gf.Vec3d(0, 0, self.start_offset - sample["fallen_m"]))
        status = "FIRST CONTACT - IMPACT NOT SOLVED" if sample["at_first_contact"] else "FREE FALL - PRE-IMPACT ONLY"
        self.telemetry.text = (f"{status}\nPhysical time {self.elapsed*1000:.2f} ms | foot gap {sample['clearance_m']*1000:.2f} mm\n"
                               f"Incident speed {sample['incident_downward_speed_m_s']:.3f} m/s | energy {sample['kinetic_energy_j']*1000:.2f} mJ")
        self.legend.text = (f"{self.case.height_m*1000:g} mm RELEASE | {self.case.playback_speed:g}x playback | {status}\n"
                            f"t = {self.elapsed*1000:.2f} ms   |   gap = {sample['clearance_m']*1000:.2f} mm\n"
                            "SURVIVAL NOT EVALUATED - no impact stress, bounce or failure model")
        if sample["at_first_contact"]:
            self.running = False
            self.feedback.text = "Stopped at first contact. Replay to record again, or Return to cable FEM."
        self.frames += 1

    def replay(self):
        if self.stage != omni.usd.get_context().get_stage() or not self.root.IsValid():
            self.feedback.text = "Scene changed. Return to cable FEM and reconnect the matching scene."
            return
        self.set_time(0.0)
        self.previous = time.perf_counter()
        self.running = True
        self.feedback.text = "Falling at 0.05x playback. Automatic hard stop at first contact."

    def toggle_pause(self):
        if self.elapsed >= self.case.contact_time_s:
            self.feedback.text = "Contact reached; use Replay 70 mm. No post-impact solution exists."
            return
        self.running = not self.running
        self.previous = time.perf_counter()
        self.feedback.text = "Fall resumed." if self.running else "Fall paused."

    async def loop(self):
        self.previous = time.perf_counter()
        try:
            while not self.closed:
                await omni.kit.app.get_app().next_update_async()
                now = time.perf_counter()
                delta, self.previous = now - self.previous, now
                if self.stage != omni.usd.get_context().get_stage() or not self.root.IsValid():
                    self.running = False
                    self.feedback.text = "Scene changed; preview halted. Return and reconnect the exact knee."
                    continue
                if self.running:
                    self.set_time(self.elapsed + delta * self.case.playback_speed)
        except asyncio.CancelledError:
            pass

    def save(self):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        folder = self.lab.project / "exact_joint/results" / (stamp + "_drop")
        folder.mkdir(parents=True, exist_ok=False)
        report = self.case.report()
        report["source_sha256"] = self.lab.model.source["sha256"]
        report["current_frame"] = self.case.sample(self.elapsed)
        (folder / "drop.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        self.feedback.text = "Saved assumptions/ballistics only: " + folder.name
        return folder

    async def restore(self):
        if self.closed:
            return
        self.closed = True
        self.running = False
        if self.task:
            self.task.cancel()
            await self.task
        if self.op and self.root.IsValid():
            order = self.root.GetAttribute("xformOpOrder")
            if self.original_order_authored:
                order.Set(self.original_order)
            else:
                self.root.RemoveProperty("xformOpOrder")
            self.root.RemoveProperty("xformOp:translate:dropPreview")
            self.root.SetTypeName(self.original_type)
        if self.diagnostic and self.diagnostic.GetPrim().IsValid():
            self.diagnostic.GetPrim().SetActive(self.diagnostic_active)
            self.diagnostic.GetVisibilityAttr().Set(self.diagnostic_visibility)
        if self.hud:
            self.hud.clear()
        if self.window:
            self.window.visible = False
            self.window.destroy()
        self.lab.window.visible = True
        self.lab.window.focus()
        self.lab.drop_preview = None
        self.lab.timeline_was_playing = app_utils.is_playing()
        if self.stage == omni.usd.get_context().get_stage() and self.root.IsValid():
            self.lab.live_display = LiveFemDisplay(self.lab.scene)
            self.lab.calculate()
            self.lab.paused = True
            self.lab.compare_view()
            self.lab.feedback.text = "Returned to unloaded cable FEM. Drop survival was NOT evaluated."
        else:
            self.lab.reject_command("Scene changed during preview; reopen the saved knee and Reconnect opened knee.")
        self.lab.task = asyncio.ensure_future(self.lab.loop())
