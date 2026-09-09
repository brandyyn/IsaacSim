"""Live contact-force, bend and FEM-stress display for the guarded impact model."""

from __future__ import annotations

import asyncio
import dataclasses
import json
import time
from datetime import datetime, timezone

import numpy as np
import omni.kit.app
import omni.ui as ui
import omni.usd
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.experimental.utils import app as app_utils
from pxr import Gf, UsdGeom
from scipy.spatial.transform import Rotation

from exact_joint.drop_preview import DropPreview
from exact_joint.drop_test import DropCase
from exact_joint.impact import FemImpact, ImpactConfig
from exact_joint.live_display import LiveFemDisplay
from exact_joint.scene import curves, material, set_points


class ImpactPreview(DropPreview):
    """Reuse reversible preview ownership, but integrate actual post-contact forces."""

    def __init__(self, lab) -> None:
        super().__init__(lab)
        self.impact = FemImpact(lab.model)
        self.impact_display = None
        self.pose_op = None
        self.force_arrows = None
        self.in_contact_phase = False
        self.impact_budget_s = 0.0
        self.gain = 500.0
        self.impact_playback = .0001
        self.auto_closeup = True
        self.timeline_was_playing = False

    async def start(self) -> None:
        await super().start()
        self.pose_op = UsdGeom.Xformable(self.root).AddTransformOp(opSuffix="impactPose")
        self.pose_op.Set(Gf.Matrix4d(1))
        self.diagnostic.GetPrim().SetActive(True)
        self.diagnostic.MakeVisible()
        self.impact_display = LiveFemDisplay(self.lab.scene)
        self.impact_display.close()  # Use the impact legend, not the cable-force legend.
        self.force_arrows = curves(self.stage, "/World/ExactFemDisplay/ImpactForceArrows", 4, .0004,
                                   material(self.stage, "ImpactForceOrange", (1, .3, .01)))
        self.timeline_was_playing = app_utils.is_playing()
        self.render_impact()

    def build_ui(self) -> None:
        # Build the base HUD, then replace only our own control window content.
        super().build_ui()
        self.window.title = "Drop impact - FEM-derived response"
        with self.window.frame, ui.VStack(spacing=5):
            ui.Label("DROP -> CONTACT FORCE -> KNEE BEND", height=26, style={"font_size": 19})
            ui.Label("EXPLORATORY REDUCED FEM DYNAMICS\nUnconverged; not a survival/failure test", height=42,
                     style={"color": 0xFF80DFFF, "font_size": 17})
            with ui.HStack(height=28):
                ui.Button("Run / replay impact", clicked_fn=self.replay)
                ui.Button("Pause / resume", clicked_fn=self.toggle_pause)
            with ui.HStack(height=28):
                ui.Button("Whole leg", clicked_fn=self.whole_view)
                ui.Button("Impact close-up", clicked_fn=self.close_view)
                ui.Button("Update gain only", clicked_fn=self.update_gain)
            with ui.HStack(height=28):
                ui.Button("Save force/bend/FEM trace", clicked_fn=self.save)
                ui.Button("Return to cable FEM", clicked_fn=lambda: asyncio.ensure_future(self.restore()))
            self.display_note = ui.Label("", height=43, word_wrap=True)
            self.telemetry = ui.Label("", height=116, word_wrap=True)
            self.feedback = ui.Label("Ready. Run recomputes from your inputs; angles are never scripted.", height=58, word_wrap=True)
            with ui.ScrollingFrame(), ui.VStack(spacing=6, height=0):
                ui.Label("INPUTS: change, then Run / replay impact to apply", height=26)
                self.inputs = {}
                for label, value in (("Drop height (mm)", 70), ("Upper-side mass (g)", 30),
                                     ("Lower-side mass (g)", 20), ("Contact stiffness (N/m)", 20000),
                                     ("Contact damping (N s/m)", .1), ("Diagnostic gain (1-1000)", 500)):
                    with ui.HStack(height=24):
                        ui.Label(label, width=230)
                        field = ui.FloatField()
                        field.model.set_value(value)
                        self.inputs[label] = field.model
                ui.Label("Mass/inertia and contact values are assumptions.\nMaterial changes: Return -> edit -> Rebuild -> reopen impact.", height=43)
                with ui.HStack(height=26):
                    ui.Label("Auto close-up at contact", width=230)
                    box = ui.CheckBox()
                    box.model.set_value(True)
                    box.model.add_value_changed_fn(lambda model: setattr(self, "auto_closeup", model.as_bool))
                ui.Label("Free fall 0.05x; impact 0.0001x playback (10,000x slower).\nStop at 1% principal strain or 2 deg small-motion limit.\nNo large-fold, fracture, bond-failure or stress-wave prediction.\nNative Play/Pause also controls this custom solver, not PhysX FEM.", height=96, word_wrap=True)

    def whole_view(self) -> None:
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[.36, -.55, .33], target=[.025, 0, .12])

    def close_view(self) -> None:
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[.10, -.17, .175], target=[.032, 0, .111])

    def update_gain(self) -> None:
        gain = self.inputs["Diagnostic gain (1-1000)"].as_float
        if not np.isfinite(gain) or not 1 <= gain <= 1000:
            self.feedback.text = "Gain must be 1-1000. No force or FEM result was changed."
            return
        self.gain = gain
        self.render_impact()

    def replay(self) -> None:
        if self.stage != omni.usd.get_context().get_stage() or not self.root.IsValid():
            self.feedback.text = "Scene changed. Return and reconnect the matching exact knee."
            return
        try:
            config = dataclasses.replace(
                self.impact.config, height_m=self.inputs["Drop height (mm)"].as_float / 1000,
                upper_mass_kg=self.inputs["Upper-side mass (g)"].as_float / 1000,
                lower_mass_kg=self.inputs["Lower-side mass (g)"].as_float / 1000,
                contact_stiffness_n_m=self.inputs["Contact stiffness (N/m)"].as_float,
                contact_damping_ns_m=self.inputs["Contact damping (N s/m)"].as_float,
            )
            candidate = FemImpact(self.lab.model, config)
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            self.feedback.text = "NOT APPLIED: " + str(exc)
            return
        self.impact = candidate
        self.case = DropCase(config.height_m, config.upper_mass_kg, config.lower_mass_kg)
        self.start_offset = self.case.height_m - self.original_clearance
        self.in_contact_phase = False
        self.impact_budget_s = 0.0
        self.elapsed = 0.0
        self.pose_op.Set(Gf.Matrix4d(1))
        self.update_gain()
        self.set_time(0.0)
        self.whole_view()
        self.previous = time.perf_counter()
        self.running = True
        self.feedback.text = "Running: gravity release, then live contact/FEM integration."

    def set_time(self, elapsed: float) -> None:
        # Called by the base initializer too, before display/pose handles exist.
        sample = self.case.sample(elapsed)
        self.elapsed = sample["time_s"]
        self.op.Set(Gf.Vec3d(0, 0, self.start_offset - sample["fallen_m"]))
        if self.impact_display:
            self.render_impact()
        else:
            self.legend.text = "FEM-DERIVED IMPACT | actual 1x; diagnostic 500x | UNVALIDATED"
        if sample["at_first_contact"] and not self.in_contact_phase:
            self.in_contact_phase = True
            if self.auto_closeup:
                self.close_view()
        self.frames += 1

    def toggle_pause(self) -> None:
        if self.impact.stopped:
            self.feedback.text = "Model/time limit reached. Change inputs and Run to start a new case."
            return
        self.running = not self.running
        self.previous = time.perf_counter()
        self.feedback.text = "Resumed." if self.running else "Paused: force, pose and stress are held."

    def render_impact(self) -> None:
        if self.impact_display is None:
            return
        result = self.impact.recover()
        self.lab.scene.update({"Knee": result}, show_stress=True, stress_scale_pa=4e7)
        # Reproject only the physical rigid roofs using exact transforms.
        # FEM nodes still use linear constraint modes; mismatch is O(angle^2).
        source = self.lab.model.source
        roof_points = source["points"].copy()
        lower = source["bottom"]
        roof_points[lower] = roof_points[lower] @ result["rotation"].T + result["q"][:3]
        roof_points += [0, 0, self.lab.scene.top_height - source["height_m"]]
        set_points(self.lab.scene.modules["Knee"]["roofs"], roof_points)
        upper = self.impact.position[:6]
        rotation = Rotation.from_rotvec(upper[3:]).as_matrix()
        pivot = np.array([0, 0, self.lab.scene.top_height])
        matrix = np.eye(4)
        matrix[:3, :3] = rotation.T
        matrix[3, :3] = pivot + upper[:3] - rotation @ pivot
        if self.pose_op:
            self.pose_op.Set(Gf.Matrix4d(*matrix.ravel().tolist()))
        self.impact_display.update(result, self.gain, 4e7, True)
        self.display_note.text = (f"Actual leg 1x; right knee: displacement {self.gain:g}x ONLY.\n"
                                  "Orange ground-force lines: 2 mm/N; cables unloaded.")
        row = self.impact.trace[-1]
        stats = result["stats"]
        phase = "LIVE EARLY IMPACT" if self.in_contact_phase else "FREE FALL - NO CONTACT YET"
        if self.impact.stopped:
            phase = "STOPPED: MODEL LIMIT" if "LIMIT" in self.impact.stop_reason else "TIME WINDOW COMPLETE"
        force = row["ground_force_n"] if self.in_contact_phase else 0.0
        self.legend.text = (f"{phase} | ACTUAL LEG 1x | RIGHT: displacement {self.gain:g}x ONLY\n"
                            f"Ground {force:.3f} N | true bend Y {stats['bend_xy_deg'][1]:.5f} deg | "
                            f"contact +{self.impact.time_s*1000:.4f} ms\n"
                            "Unconverged reduced FEM; colors 0-40+ MPa, NOT failure. No survival verdict.")
        self.telemetry.text = (
            f"Ground force: {force:.4f} N | physical contact time +{self.impact.time_s*1000:.4f} ms\n"
            f"Bend X / Y: {stats['bend_xy_deg'][0]:.5f} / {stats['bend_xy_deg'][1]:.5f} deg\n"
            f"Compression: {stats['compression_m']*1e6:.3f} um | twist: {stats['twist_deg']:.5f} deg\n"
            f"PET / PLA peak: {stats['pet_peak_pa']/1e6:.3f} / {stats['pla_peak_pa']/1e6:.3f} MPa\n"
            f"Principal strain: {stats['max_principal_strain']*100:.4f}% | FEM step: {self.impact.config.step_s*1e6:g} us\n"
            f"Energy balance error: {row['relative_energy_error']*100:.5f}%"
        )
        if self.force_arrows:
            points = []
            lower_cap = np.array([0, 0, self.lab.scene.top_height - source["height_m"]])
            for point, load in zip(self.impact.contact_points, row["corner_forces_n"]):
                local = point @ result["rotation"].T + result["q"][:3] + lower_cap
                world = rotation @ (local - pivot) + pivot + upper[:3] + np.array(self.op.Get())
                length = .002 * load if self.in_contact_phase else 0.0
                points.extend([world, world + [0, 0, length]])
            set_points(self.force_arrows, np.asarray(points))
            self.force_arrows.MakeVisible() if force > 0 else self.force_arrows.MakeInvisible()

    async def loop(self) -> None:
        self.previous = time.perf_counter()
        try:
            while not self.closed:
                await omni.kit.app.get_app().next_update_async()
                now = time.perf_counter()
                delta, self.previous = now - self.previous, now
                if self.stage != omni.usd.get_context().get_stage() or not self.root.IsValid():
                    self.running = False
                    self.feedback.text = "Scene changed; impact stopped. Return to reconnect."
                    continue
                playing = app_utils.is_playing()
                if playing != self.timeline_was_playing:
                    self.timeline_was_playing = playing
                    if playing and (self.impact.stopped or self.elapsed == 0):
                        self.replay()
                    else:
                        self.running = playing
                if not self.running:
                    continue
                if not self.in_contact_phase:
                    self.set_time(self.elapsed + min(delta, .1) * self.case.playback_speed)
                    continue
                self.impact_budget_s += min(delta, .1) * self.impact_playback
                changed = False
                while self.impact_budget_s >= self.impact.config.step_s and not self.impact.stopped:
                    self.impact.advance()
                    self.impact_budget_s -= self.impact.config.step_s
                    changed = True
                if changed:
                    self.render_impact()
                    self.frames += 1
                if self.impact.stopped:
                    self.running = False
                    self.feedback.text = (self.impact.stop_reason + "\nThis is NOT fracture. "
                                          "No later peak, rebound or survival prediction is available.")
        except asyncio.CancelledError:
            pass
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            self.running = False
            self.feedback.text = "Impact solver stopped: " + str(exc)

    def save(self):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        folder = self.lab.project / "exact_joint/results" / (stamp + "_impact")
        folder.mkdir(parents=True, exist_ok=False)
        report = self.impact.report()
        report["display_only_gain"] = self.gain
        report["current_phase"] = "impact" if self.in_contact_phase else "precontact"
        (folder / "impact.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        self.feedback.text = "Saved physical SI force/bend/stress trace and assumptions: " + folder.name
        return folder

    async def restore(self) -> None:
        if self.closed:
            return
        self.running = False
        if self.pose_op and self.root.IsValid():
            # The parent restores the original op order and its owned translate.
            self.root.RemoveProperty("xformOp:transform:impactPose")
        if self.force_arrows and self.force_arrows.GetPrim().IsValid():
            self.stage.RemovePrim("/World/ExactFemDisplay/ImpactForceArrows")
        if self.impact_display:
            self.impact_display.close()
        await super().restore()
