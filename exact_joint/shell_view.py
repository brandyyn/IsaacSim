"""Opt-in nonlinear perimeter-frame knee workshop in the existing Isaac stage."""

import asyncio
import copy
import dataclasses
import json
import math
import time
from datetime import datetime, timezone

import numpy as np
import omni.kit.app
import omni.ui as ui
import omni.usd
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.experimental.utils import app as app_utils
from pxr import Gf, UsdGeom, UsdShade, Vt
from scipy.spatial.transform import Rotation

from exact_joint.drop_preview import DropPreview
from exact_joint.mechanics import CABLE_FAMILIES, CABLE_FAMILY_NOTES, tension_pattern
from exact_joint.nonlinear_shell import NonlinearShell, ShellConfig
from exact_joint.scene import ExactLegScene, curves, material, mesh, set_points
from exact_joint.shell_impact import ShellImpact, ShellImpactConfig
from exact_joint.shell_actuation import winch_pull
from exact_joint.shell_ui_jobs import ShellJobControls
from exact_joint.shell_displacement_view import ShellDisplacementView
from exact_joint.shell_display_math import frame_blend_display
from exact_joint.shell_presentation import SolvedTransition
from exact_joint.stl_assets import STL_TO_METRE, load_joint_stl_assets


class _MovementProxy:
    """Programmatic compatibility handle for a movement without a UI button."""
    def __init__(self, view, family, enabled=True):
        self.view, self.family = view, family
        self.enabled = enabled
        self.visible = True

    def call_clicked_fn(self):
        if not self.enabled:
            return None
        return self.view.start_movement(self.family)


class ShellWorkshop(ShellJobControls, DropPreview):
    def __init__(self, lab):
        super().__init__(lab)
        self.shell = None
        self.init_job_controls()
        self.owned_path = "/World/NonlinearFrameLeg"
        self.owned = None
        self.state = None
        self.mode = "NEUTRAL"
        self.offset = lab.scene.top_height-lab.model.source["height_m"]
        self.drop = None
        self.tensions = np.zeros(12)
        self.snapshots = []
        self.last_rejection = None
        self.saved_path = None
        self.timeline_was_playing = False
        self.display_index = None
        self.static_trace = []
        self.timeline_action = "manual"
        self.show_response = True
        self.response_gain = 100.0
        self.inspector = None
        self.scene_disconnected = False
        self.smooth_motion = True
        self.strain_colours = False
        self.presentation = None
        self.presentation_clock = time.perf_counter()
        self.presentation_frames = 0
        self.accepted_report = None
        self.active_family = None
        self.active_cable_mask = np.zeros(12, dtype=bool)
        self.last_reaction = np.zeros(6)
        self.requested_displacement_mm = 0.0
        self.stl_reference = None
        self.stl_layout = None
        self.stl_reference_visible = False
        self.stl_layout_visible = False
        self.stl_display_meshes = []

    async def start(self):
        config = ShellConfig(sparse_solver=True, interior_refinement=0,
                             physical_strip_bending=True, self_contact=True, crease_twist_ratio=0,
                             max_iterations=900,
                             frame_hinge_width_m=.0002, open_ends=True)
        self.shell = await asyncio.to_thread(NonlinearShell, self.lab.model.source, self.lab.config, config)
        self.state = np.zeros(self.shell.ndof)
        await super().start()
        self.original_leg_active = self.root.IsActive()
        self.root.SetActive(False)
        self.owned = UsdGeom.Xform.Define(self.stage, self.owned_path)
        self.owned.GetPrim().SetCustomDataByKey("source_sha256", self.shell.source["sha256"])
        self.owned.GetPrim().SetCustomDataByKey("scope", "Nonlinear membrane/discrete-hinge experiment; rigid perimeter frames only")
        self.make_scene()
        self.render()
        self.whole_view()

    def build_ui(self):
        self._build_simple_ui()
        return

    def _build_simple_ui(self):
        """Build the compact single-joint cable/FEM control surface."""
        self.window = ui.Window("Joint FEM - cable folding pattern", width=560, height=520)
        self.inputs = {}
        families = CABLE_FAMILIES
        with self.window.frame, ui.ScrollingFrame(), ui.VStack(spacing=5, height=0):
            ui.Label("SINGLE JOINT | PHOTO/VIDEO CABLE FOLDING FEM", height=28, style={"font_size": 18})
            ui.Label("This restores the original seven-family cable pattern: four axial strands for compression, paired corner strands for bend, and crossed strands for twist. The square frames stay rigid; the triangular PLA-on-PET fold lines deform. Each cable pull runs an accepted FEM solve.", height=72, word_wrap=True,
                     style={"color": 0xFF80DFFF})
            with ui.HStack(height=30):
                ui.Label("Cable family", width=120)
                self.cable_pattern = ui.ComboBox(0, *families)
                ui.Button("Apply cable force", clicked_fn=lambda: self.start_movement(self._selected_family()),
                          tooltip="Read the force below and solve a four-step cable loading ramp for the selected family.")
                ui.Button("Neutral", clicked_fn=self.neutral_shell)
                ui.Button("Reconnect scene", clicked_fn=self.reconnect_opened,
                          tooltip="After opening a matching saved exact-knee USD, rebind the FEM overlay without saving over it.")
            with ui.HStack(height=28):
                ui.Button("Show STL assembled", clicked_fn=self.toggle_stl_reference,
                          tooltip="Show the supplied CAD below the FEM joint. Its display motion blends the accepted FEM end-frame poses; it is not a solid FEM result.")
                ui.Button("Show flat Rigid", clicked_fn=self.toggle_stl_layout,
                          tooltip="Toggle the supplied Rigid.stl manufacturing layout; it is not an assembled FEM body.")
            self.stl_status = ui.Label("STL reference loaded | assembled hidden", height=24, word_wrap=True,
                                       style={"color": 0xFF80DFFF})
            with ui.HStack(height=30):
                ui.Label("Cable tension (N / active strand)", width=220)
                target = ui.FloatField(width=90).model
                target.set_value(3.0)
                self.inputs["Cable tension (N)"] = target
                ui.Label("Use 0.5–3 N first. Tension is a load, not an angle command; the FEM decides the fold.", word_wrap=True)
            self.pattern_note = ui.Label("", height=26, word_wrap=True, style={"color": 0xFF80DFFF})
            self.cable_pattern.model.add_item_changed_fn(lambda *_: self._selected_family())
            self.progress_label = ui.Label("READY | choose a cable family", height=29, word_wrap=True, style={"color": 0xFF80DFFF})
            self.presentation_label = ui.Label("", height=28, word_wrap=True, style={"color": 0xFF80DFFF})
            with ui.HStack(height=26):
                self.response_visible = ui.CheckBox(width=22).model
                self.response_visible.set_value(self.show_response)
                self.response_visible.add_value_changed_fn(lambda m: self.change_response(visible=m.as_bool))
                ui.Label("Display-only displacement scale", width=210)
                self.response_gain_input = ui.FloatField(width=65).model
                self.response_gain_input.set_value(self.response_gain)
                self.response_gain_input.add_value_changed_fn(lambda m: self.change_response(gain=m.as_float))
            self.response_label = ui.Label("", height=38, word_wrap=True)
            self.telemetry = ui.Label("", height=98, word_wrap=True)
            self.settings_label = ui.Label("", height=55, word_wrap=True)
            self.feedback = ui.Label("Ready. Cable forces follow the photo/video routing; FEM reaction and deformation are reported after each solve.", height=54, word_wrap=True)
            with ui.CollapsableFrame("Material / fold stiffness", collapsed=True):
                ui.Label("Assumed PLA/PET properties; changing them requires Rebuild model.", height=28, word_wrap=True)
                for label, value in (("Fold compliance (100 = PET reference)", 100),
                                     ("Panel bending scale", self.shell.config.panel_bending_scale),
                                     ("Membrane stiffness scale", self.shell.config.membrane_scale),
                                     ("PLA modulus (GPa)", self.shell.material.pla_modulus_pa/1e9),
                                     ("PET modulus (GPa)", self.shell.material.pet_modulus_pa/1e9),
                                     ("PET hinge width (mm)", self.shell.config.frame_hinge_width_m*1000)):
                    with ui.HStack(height=25):
                        ui.Label(label, width=255)
                        field = ui.FloatField(width=90).model
                        field.set_value(value)
                        self.inputs[label] = field
                ui.Button("Rebuild model", clicked_fn=lambda: self.launch(self.rebuild(), "Rebuild model"))
            ui.Label("Custom nonlinear membrane/discrete-hinge shell. No solid stress, failure or survival rating is implied; magnification is display-only.", height=34, word_wrap=True)
            self.legend = ui.Label("", height=42, word_wrap=True, style={"color": 0xFF80DFFF})
        self.active_family = families[0]
        self.pattern_note.text = "Pattern: " + CABLE_FAMILY_NOTES[families[0]]
        # Compatibility handles keep scripted validation possible without
        # adding seven redundant buttons to the user-facing panel.
        self.movement_buttons = {family: _MovementProxy(self, family) for family in families}
        self.roof_button = _MovementProxy(self, "Roof flex test", enabled=False)
        self.pose_buttons = []
        self.refresh_pose_controls()

    def _selected_family(self):
        families = CABLE_FAMILIES
        index = self.cable_pattern.model.get_item_value_model().as_int
        family = families[index]
        if hasattr(self, "pattern_note"):
            self.pattern_note.text = "Pattern: " + CABLE_FAMILY_NOTES[family]
        return family

    def _legacy_build_ui(self):
        """Retained source for the previous panel; never shown."""
        self.window.title = "Nonlinear knee - rigid frames / flexible panels"
        # The dock may be shorter than the fixed control group. Scroll the
        # whole form so material/hinge inputs never collapse to zero height.
        with self.window.frame, ui.ScrollingFrame() as self.form_scroll, ui.VStack(spacing=3, height=0):
            ui.Label("OPEN FRAMES | PLA PANELS + PET HINGES", height=22, style={"font_size": 18})
            ui.Label("EXPERIMENTAL SHELL FEM - not validated material capacity\n1x geometry. Smooth transitions are DISPLAY ONLY, not solved motion.", height=33,
                     style={"color": 0xFF80DFFF})
            with ui.HStack(height=29):
                ui.Button("Drop 70 mm / recompute", clicked_fn=self.start_drop)
                ui.Button("Pause / resume", clicked_fn=self.pause_jobs)
                ui.Button("Neutral / cancel", clicked_fn=self.neutral_shell)
            self.progress_label = ui.Label("READY | Choose a cable movement below", height=30, word_wrap=True,
                                           style={"color": 0xFF80DFFF})
            with ui.HStack(height=25):
                smooth = ui.CheckBox(width=22).model
                smooth.set_value(self.smooth_motion)
                smooth.add_value_changed_fn(lambda m: self.change_presentation(smooth=m.as_bool))
                ui.Label("Smooth solved transitions", width=205)
                strain = ui.CheckBox(width=22).model
                strain.set_value(self.strain_colours)
                strain.add_value_changed_fn(lambda m: self.change_presentation(strain=m.as_bool))
                ui.Label("Strain colours (accepted frames)")
            self.presentation_label = ui.Label("", height=29, word_wrap=True, style={"color": 0xFF80DFFF})
            with ui.HStack(height=25):
                self.response_visible = ui.CheckBox(width=22).model
                self.response_visible.set_value(self.show_response)
                self.response_visible.add_value_changed_fn(lambda m: self.change_response(visible=m.as_bool))
                ui.Label("Magnified response", width=145)
                self.response_gain_input = ui.FloatField(width=65).model
                self.response_gain_input.set_value(self.response_gain)
                self.response_gain_input.add_value_changed_fn(lambda m: self.change_response(gain=m.as_float))
                ui.Button("Replay response", clicked_fn=lambda: self.inspector.replay())
                ui.Button("Hold", clicked_fn=lambda: self.inspector.stop())
            self.response_label = ui.Label("", height=45, word_wrap=True, style={"color": 0xFF80DFFF})
            self.movement_buttons = {}
            for families in (("Compression", "Bend X+", "Bend X-"), ("Bend Y+", "Bend Y-", "Twist CW", "Twist CCW")):
                with ui.HStack(height=29):
                    for family in families:
                        self.movement_buttons[family] = ui.Button(family, clicked_fn=lambda f=family: self.start_movement(f),
                            tooltip="Apply the selected cable tension to this pattern. Force-driven; no target angle is imposed.")
            self.inputs = {}
            with ui.HStack(height=24):
                ui.Label("Cable tension (N / active strand)", width=240)
                field = ui.FloatField()
                field.model.set_value(3)
                self.inputs["Cable tension (N)"] = field.model
            with ui.HStack(height=29):
                self.cable_pattern = ui.ComboBox(0, "Compression", "Bend X+", "Bend X-", "Bend Y+", "Bend Y-", "Twist CW", "Twist CCW")
                ui.Button("Apply cable + force", clicked_fn=lambda: self.start_range("manual"))
            with ui.HStack(height=29):
                ui.Button("Cable demo", clicked_fn=lambda: self.start_range("cables"))
                self.roof_button = ui.Button("Roof flex test", clicked_fn=lambda: self.start_range("roof"))
                ui.Button("Joint view", clicked_fn=self.whole_view)
                ui.Button("Knee close-up", clicked_fn=self.close_view)
            with ui.HStack(height=29):
                ui.Button("Apply winch pull", clicked_fn=lambda: self.start_range("winch"))
                ui.Button("Winch demo", clicked_fn=lambda: self.start_range("winch_demo"))
                ui.Button("Material reference", clicked_fn=lambda: self.launch(self.preset(False), "Material reference"))
                ui.Button("Relief experiment", clicked_fn=lambda: self.launch(self.preset(True), "Relief experiment"))
            with ui.HStack(height=29):
                ui.Button("Rebuild stiffness", clicked_fn=lambda: self.launch(self.rebuild(), "Rebuild stiffness"))
                ui.Button("Reconnect opened knee", clicked_fn=self.reconnect_opened,
                          tooltip="Bind the matching opened USD using the current material/mesh settings. Resets only the owned knee overlay to neutral; does not save the USD.")
                ui.Button("Save calculation", clicked_fn=self.save)
                ui.Button("Return to reference FEM", clicked_fn=lambda: asyncio.ensure_future(self.restore()))
            with ui.HStack(height=29):
                ui.Button("Photo joint", width=100, clicked_fn=lambda: self.launch(self.preset(False, open_ends=True), "Photo joint"),
                          tooltip="Open ends and PET frame-to-side-panel hinges. Keeps JSON side geometry; does not invent measured lug or hole dimensions.")
                ui.Button("Capped comparison", width=140, clicked_fn=lambda: self.launch(self.preset(False, frame_hinges=True), "Capped comparison"),
                          tooltip="Rebuild both roofs with a 0.2 mm PET flexure border. PLA 0.4 mm / PET 80 um; no artificial material softening.")
                ui.Label("PET hinge (mm)", width=120)
                self.inputs["Frame-plate PET hinge (mm)"] = ui.FloatField(width=65).model
                self.inputs["Frame-plate PET hinge (mm)"].set_value(self.shell.config.frame_hinge_width_m*1000)
                ui.Button("Apply hinge width", clicked_fn=lambda: self.launch(self.rebuild(), "Apply hinge width"),
                          tooltip="Apply frame hinge width and all pending material/mesh inputs. Green outlines mark PET/PLA interfaces.")
            with ui.HStack(height=27):
                ui.Button("Smooth replay solved states", clicked_fn=lambda: self.launch(self.replay_recorded(), "Solved-state replay"))
                ui.Button("Show peak bending", clicked_fn=self.show_peak)
            self.telemetry = ui.Label("", height=98, word_wrap=True)
            self.settings_label = ui.Label("", height=59, word_wrap=True)
            self.feedback = ui.Label("Apply and Cable demo solve forces; Drop solves impact. Full travel is not guaranteed by a force input.", height=44, word_wrap=True)
            with ui.CollapsableFrame("Advanced FEM settings / material / drop", collapsed=True):
                ui.Label("INPUTS: stiffness changes require Rebuild", height=25)
                self.flags = {}
                for label, value in (("Thickness-derived PET bending (100 = PET reference)", self.shell.config.physical_strip_bending),
                                     ("Sparse element solver", self.shell.config.sparse_solver),
                                     ("Open ends (photo design, no JSON caps)", self.shell.config.open_ends),
                                     ("Midsurface self-contact + CCD", self.shell.config.self_contact)):
                    with ui.HStack(height=24):
                        field = ui.CheckBox(width=24)
                        field.model.set_value(value)
                        self.flags[label] = field.model
                        ui.Label(label)
                with ui.HStack(height=24):
                    ui.Label("Boundary subdivision (1-5)", width=240)
                    self.boundary_input = ui.IntField().model
                    self.boundary_input.set_value(self.shell.config.subdivision)
                with ui.HStack(height=24):
                    ui.Label("Interior refinement (0/1/2)", width=240)
                    self.refinement_input = ui.IntField().model
                    self.refinement_input.set_value(self.shell.config.interior_refinement)
                for label, value in (("Fold compliance (100 = PET reference)", 100), ("Crease twist coupling", self.shell.config.crease_twist_ratio),
                                     ("Panel bending scale", 1), ("Membrane stiffness scale", 1),
                                     ("PLA thickness (mm)", self.shell.material.pla_thickness_m*1000),
                                     ("PET thickness (um)", self.shell.material.pet_thickness_m*1e6),
                                     ("PLA modulus (GPa, assumed)", self.shell.material.pla_modulus_pa/1e9),
                                     ("PET modulus (GPa, assumed)", self.shell.material.pet_modulus_pa/1e9),
                                     ("PET junction relief (%)", 0), ("Winch pull (mm)", .5),
                                     ("Winch stiffness (N/m)", 1000), ("Winch force cap (N)", 10),
                                     ("Static solver iterations", 900),
                                     ("Membrane strain guard (%)", 3), ("Roof test force (N)", 1),
                                     ("Lower frame force X (N)", 0), ("Lower frame force Y (N)", 0),
                                     ("Lower frame force Z (N)", 0), ("PET exposed gap (mm)", self.lab.config.hinge_gap_m*1000),
                                     ("Upper-side mass (g)", 30), ("Lower-side mass (g)", 20),
                                     ("Contact stiffness (N/m)", 20000), ("Contact damping (N s/m)", .1),
                                     ("Drop timestep (us)", 25), ("Drop duration (ms)", 15)):
                    with ui.HStack(height=24):
                        ui.Label(label, width=240)
                        field = ui.FloatField()
                        field.model.set_value(value)
                        self.inputs[label] = field.model
                ui.Label("Ratios/scales are uncalibrated experiments, not new material data.\n"
                         "Ranges stop on strain, separation, intersection or convergence guards.\n"
                         "IPC checks midsurface paths; finite-thickness clearance is NOT modelled.\n"
                         "No fracture, delamination, cable-guide contact or strength rating.\n"
                         "Drop: assumed nodal/rigid inertia; backward Euler adds numerical damping.",
                         height=115, word_wrap=True)
                self.pose_buttons = []
                with ui.CollapsableFrame("Advanced prescribed-pose studies (NOT cable motion)", collapsed=True):
                    with ui.VStack(spacing=4):
                        self.pose_note = ui.Label("", height=42, word_wrap=True)
                        for label, value in (("Bend target (deg)", 30), ("Twist target (deg)", 60), ("Compression target (%)", 70)):
                            with ui.HStack(height=24):
                                ui.Label(label, width=240)
                                field = ui.FloatField()
                                field.model.set_value(value)
                                self.inputs[label] = field.model
                        with ui.HStack(height=29):
                            for title, mode in (("Bend range", "bend"), ("Twist range", "twist"), ("Compression range", "compression")):
                                self.pose_buttons.append(ui.Button(title, clicked_fn=lambda m=mode: self.start_range(m),
                                    tooltip="Unavailable with IPC self-contact. These impose frame displacement; use the cable buttons above."))
                self.refresh_pose_controls()

    def refresh_pose_controls(self):
        if not hasattr(self, "pose_note"):
            return
        self.roof_button.enabled = not self.shell.config.open_ends
        self.roof_button.tooltip = "No roof exists in the photo design. Apply cables or lower-frame force instead." if self.shell.config.open_ends else "Load the roof centre in the capped comparison."
        enabled = self.shell.contact is None
        for button in self.pose_buttons:
            button.enabled = enabled
        self.pose_note.text = ("Prescribed displacement; NOT a prediction of cable-driven travel." if enabled else
                               "Disabled with self-contact ON. Use the Compression/Bend/Twist cable buttons at the top.")

    def scene_is_current(self):
        try:
            return (self.stage == omni.usd.get_context().get_stage()
                    and self.owned is not None and self.owned.GetPrim().IsValid())
        except (RuntimeError, TypeError):
            return False

    def job_context_valid(self):
        return self.scene_is_current()

    def ensure_current_scene(self):
        if self.scene_is_current():
            return True
        self.feedback.text = "Scene disconnected. Click Reconnect opened knee before changing the displayed result."
        return False

    def launch(self, coroutine, label="Calculation"):
        if not self.scene_is_current():
            coroutine.close()
            self.job_outcome = "NOT APPLIED: scene disconnected"
            self.feedback.text = "Click Reconnect opened knee. Controls are bound to a different scene; no calculation was started."
            return
        super().launch(coroutine, label)

    def reconnect_opened(self):
        """Explicit recovery, never silently attach a different robot or import code from USD."""
        if self.work is not None and not self.work.done():
            self.feedback.text = "Cancel the active calculation and wait for its worker before reconnecting."
            return
        try:
            self.rebind_scene(omni.usd.get_context().get_stage())
        except Exception as exc:
            self.job_outcome = "NOT RECONNECTED"
            self.feedback.text = "NOT RECONNECTED: " + str(exc)

    def rebind_scene(self, stage):
        if stage is None:
            raise ValueError("Open a saved exact-knee USD first.")
        overlay = stage.GetPrimAtPath(self.owned_path)
        if not overlay or overlay.GetCustomDataByKey("source_sha256") != self.shell.source["sha256"]:
            raise ValueError("Opened scene does not contain this original JSON knee overlay; nothing changed.")
        reference = stage.GetPrimAtPath("/World/ExactLeg")
        diagnostic = stage.GetPrimAtPath("/World/ExactFemDisplay")
        if not reference or not diagnostic or not stage.GetPrimAtPath("/World/Plinth"):
            raise ValueError("Saved knee is missing its reference leg, diagnostic or floor.")
        cache = UsdGeom.XformCache()
        for prim in (stage.GetPrimAtPath("/World"), reference, overlay):
            if cache.GetLocalToWorldTransform(prim) != Gf.Matrix4d(1):
                raise ValueError("Reconnect requires the original untransformed leg/world roots; nothing changed.")
        # Temporarily expose the saved reference only for read-only schema and
        # source/config validation. Restore activation even when validation fails.
        active = reference.IsActive()
        reference.SetActive(True)
        try:
            candidate = ExactLegScene.attach(stage, self.lab.model)
        finally:
            reference.SetActive(active)
        self.cancel_jobs()
        self.stage, self.root, self.lab.scene = stage, reference, candidate
        self.original_leg_active = True
        self.original_type = reference.GetTypeName()
        order = reference.GetAttribute("xformOpOrder").Get()
        self.original_order = Vt.TokenArray([name for name in (order or []) if name != "xformOp:translate:dropPreview"])
        self.original_order_authored = bool(len(self.original_order))
        attr = reference.GetAttribute("xformOp:translate:dropPreview")
        self.op = UsdGeom.XformOp(attr) if attr else None
        self.diagnostic = UsdGeom.Imageable(diagnostic)
        self.diagnostic_active = True
        self.diagnostic_visibility = UsdGeom.Tokens.inherited
        reference.SetActive(False)
        diagnostic.SetActive(False)
        # Only our hash-checked overlay is regenerated. No USD file is saved;
        # camera, lights, unrelated prims and current numerical settings survive.
        stage.RemovePrim(self.owned_path)
        self.owned = UsdGeom.Xform.Define(stage, self.owned_path)
        self.owned.GetPrim().SetCustomDataByKey("source_sha256", self.shell.source["sha256"])
        self.state = np.zeros(self.shell.ndof)
        self.snapshots, self.static_trace = [], []
        self.drop = self.last_rejection = self.display_index = None
        self.tensions = np.zeros(12)
        self.offset = self.lab.scene.top_height-self.shell.height
        self.mode = "NEUTRAL - RECONNECTED"
        self.timeline_was_playing = app_utils.is_playing()
        self.scene_disconnected = False
        self.make_scene()
        self.render()
        self.job_outcome = "READY: reconnected"
        self.job_label = "Choose a cable movement"
        self.feedback.text = "RECONNECTED. Current material/mesh settings retained; owned knee reset to neutral. USD alone is not a solver checkpoint."

    def change_response(self, visible=None, gain=None):
        if not self.ensure_current_scene():
            return
        if gain is not None:
            if not np.isfinite(gain) or not 1 <= gain <= 500:
                self.feedback.text = "Display magnification must be 1-500. It does not change the FEM."
                return
            self.response_gain = float(gain)
        if visible is not None:
            self.show_response = bool(visible)
        if self.inspector is not None:
            self.inspector.stop()
            self.render()
            if not self.show_response:
                self.response_label.text = "Magnified plot hidden. Physical leg remains at 1x."
            self.close_view()

    def start_movement(self, family):
        families = CABLE_FAMILIES
        if family not in families:
            raise ValueError("Unknown cable movement")
        self.cable_pattern.model.get_item_value_model().set_value(families.index(family))
        self.active_family = family
        # Capture inputs at the click, including queued requests. Editing the
        # dropdown or force while a worker drains must not change that request.
        amplitude = self.inputs["Cable tension (N)"].as_float
        self.launch(self.range_job("single_cable", family=family, amplitude=amplitude),
                    f"Cable {family} ({amplitude:g} N/strand)")

    def _displacement_constraints(self, family, displacement_m):
        """Map a cable take-up displacement to one lower-frame DOF.

        Translation is used for compression/bend.  Twist is the equivalent
        cable arc displacement at the half-width radius, so the solver still
        returns a reaction moment in SI units.
        """
        if family == "Compression":
            return {2: displacement_m}
        if family == "Bend X+":
            return {0: displacement_m}
        if family == "Bend X-":
            return {0: -displacement_m}
        if family == "Bend Y+":
            return {1: displacement_m}
        if family == "Bend Y-":
            return {1: -displacement_m}
        if family == "Twist CW":
            return {5: displacement_m / (self.shell.width / 2)}
        if family == "Twist CCW":
            return {5: -displacement_m / (self.shell.width / 2)}
        raise ValueError("Unknown displacement family")

    def make_scene(self):
        s, root, model = self.stage, self.owned_path, self.shell
        self.owned.GetPrim().SetCustomDataByKey("shell_config_json", json.dumps(dataclasses.asdict(model.config)))
        self.owned.GetPrim().SetCustomDataByKey("material_json", json.dumps(dataclasses.asdict(model.material)))
        self.surface = mesh(s, root+"/FlexiblePLA_PET", model.points, model.mesh["triangles"])
        self.surface.CreateDisplayColorAttr()
        UsdGeom.Primvar(self.surface.GetDisplayColorAttr()).SetInterpolation(UsdGeom.Tokens.uniform)
        white = material(s, "NonlinearCreaseWhite", (.9, .94, 1))
        gold = material(s, "NonlinearFrameGold", (.95, .55, .08))
        red = material(s, "NonlinearCableRed", (.9, .012, .025))
        black = material(s, "NonlinearCableBlack", (.012, .012, .015))
        link = material(s, "NonlinearLink", (.06, .18, .22))
        steel = material(s, "NonlinearYoke", (.65, .73, .8))
        self.line_edges = model.mesh["hinges"][model.mesh["crease_ids"] >= 0, :2]
        self.frame_edges = np.asarray([(a, b) for i in model.mesh["frame_line_ids"] for a, b in zip(model.mesh["chains"][i][:-1], model.mesh["chains"][i][1:])])
        self.lines = curves(s, root+"/Original76SubdividedCreases", len(self.line_edges), .00012, white)
        self.relief_edges = model.mesh["free_boundary_edges"].reshape(-1, 2)
        if model.config.open_ends and len(self.relief_edges):
            z = model.points[self.relief_edges, 2]
            mounting_edge = np.all(np.isclose(z, 0, atol=1e-12), axis=1) | np.all(np.isclose(z, model.height, atol=1e-12), axis=1)
            self.relief_edges = self.relief_edges[~mounting_edge]
        self.relief_lines = None
        if len(self.relief_edges):
            cyan = material(s, "ExperimentalPETRelief", (.1, .95, .8))
            self.relief_lines = curves(s, root+"/ExperimentalCutBoundaries", len(self.relief_edges), .00010, cyan)
        self.frame_hinge_edges = model.mesh.get("frame_hinge_edges", np.empty((0, 2), dtype=int))
        self.frame_hinge_lines = None
        if len(self.frame_hinge_edges):
            green = material(s, "FramePlatePETFlexure", (.08, 1, .25))
            self.frame_hinge_lines = curves(s, root+"/FramePlatePETInterfaces", len(self.frame_hinge_edges), .00007, green)
        self.frames_curve = curves(s, root+"/OnlyRigidPerimeters", len(self.frame_edges),
                                   .00025 if len(self.frame_hinge_edges) else .0007, gold)
        self.red = curves(s, root+"/CrossedActuatingCables", 8, .00022, red)
        self.axial = curves(s, root+"/AxialCables", 4, .00014, black)
        self.loaded_cables = curves(s, root+"/CurrentlyLoadedCables", 12, .0003, gold)
        self.loaded_cables.SetWidthsInterpolation(UsdGeom.Tokens.uniform)
        self.applied_force = curves(s, root+"/AppliedFrameForce", 1, .0004, gold)
        set_points(self.applied_force, np.zeros((2, 3)))
        self.applied_force.MakeInvisible()
        self.top_cross = curves(s, root+"/TopRoutingReference", 2, .00016, black)
        self.forces = curves(s, root+"/GroundForceLines", 4, .0005, gold)
        set_points(self.forces, np.zeros((8, 3)))
        self.forces.MakeInvisible()
        self.body_ops = []

        def cube(path, center, dimensions, mat):
            shape = UsdGeom.Cube.Define(s, path)
            shape.CreateSizeAttr(1)
            shape.AddTranslateOp().Set(Gf.Vec3d(*center))
            shape.AddScaleOp().Set(Gf.Vec3f(*dimensions))
            UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat)

        for frame, name in enumerate(("UpperRigidAssembly", "LowerRigidAssembly")):
            body = UsdGeom.Xform.Define(s, root+"/"+name)
            self.body_ops.append(body.AddTransformOp())
            sign = 1 if frame == 0 else -1
            if model.config.open_ends:
                # Schematic mounting bands OUTSIDE the shell ends, not roof caps.
                # Hardware dimensions are visual assumptions, excluded from FEM.
                for side in (-1, 1):
                    cube(str(body.GetPath())+f"/PhotoBandX_{side+1}", (side*model.width/2, 0, sign*.001),
                         (.0004, model.width+.0004, .002), white)
                    cube(str(body.GetPath())+f"/PhotoBandY_{side+1}", (0, side*model.width/2, sign*.001),
                         (model.width-.0004, .0004, .002), white)
            for side in (-1, 1):
                cube(str(body.GetPath())+f"/PerimeterSupport_{side+1}", (side*model.width/2, 0, sign*.004),
                     (.002, .003, .008), steel)
            cube(str(body.GetPath())+"/RaisedYoke", (0, 0, sign*.008), (model.width+.002, .004, .003), steel)
            if not model.config.open_ends:
                if frame == 0:
                    cube(str(body.GetPath())+"/Thigh", (0, 0, .046), (.009, .012, .073), link)
                else:
                    cube(str(body.GetPath())+"/Shank", (0, 0, -.042), (.009, .012, .067), link)
                    cube(str(body.GetPath())+"/Foot", (.012, 0, -.083), (.05, .026, .009), link)
        self._make_stl_reference(s)
        self.inspector = ShellDisplacementView(self)

    @staticmethod
    def _cad_points_to_model(triangles, assembled=True):
        """Map supplied CAD source units to the existing metre FEM frame.

        The CAD midsurface was verified against the JSON source: 0.2 source
        units correspond to one JSON unit, and the lower assembled end plane is
        at CAD z=6.2716503.  This is a display transform only; material
        thickness and stiffness stay in the nonlinear FEM model.
        """
        points = np.asarray(triangles, dtype=np.float64).copy()
        points[..., 0:2] *= STL_TO_METRE * 0.6
        z_base = 6.2716503143 if assembled else float(np.min(points[..., 2]))
        points[..., 2] = (points[..., 2] - z_base) * STL_TO_METRE * 0.6
        return points.reshape(-1, 3)

    def _make_stl_reference(self, stage):
        """Add the user CAD as an optional, provenance-tagged visual overlay."""
        root_path = self.owned_path + "/STLJointReference"
        root = UsdGeom.Xform.Define(stage, root_path)
        root.GetPrim().SetCustomDataByKey("source", "user-supplied STL assets")
        root.GetPrim().SetCustomDataByKey("units_assumption", "binary STL source units mapped at 0.0006 m/unit")
        root.GetPrim().SetCustomDataByKey("fem_authority", "exact_joint/source_joint.json")
        assets = load_joint_stl_assets()
        self.stl_display_meshes = []
        soft_mat = material(stage, "STLReferenceSoftPET", (.12, .85, .92), .34)
        rigid_mat = material(stage, "STLReferenceRigidPLA", (.98, .54, .10), .42)
        layout_mat = material(stage, "STLReferenceFlatRigid", (.48, .60, .95), .25)

        def add_triangles(path, triangles, mat, role, assembled=True):
            triangles = np.asarray(triangles, dtype=np.float64)
            points = self._cad_points_to_model(triangles, assembled=assembled)
            faces = [(3*i, 3*i+1, 3*i+2) for i in range(len(triangles))]
            prim = mesh(stage, path, points, faces, mat)
            prim.GetPrim().SetCustomDataByKey("role", role)
            prim.GetPrim().SetCustomDataByKey("source_units", "unverified binary STL units")
            prim.GetPrim().SetCustomDataByKey("display_scale_m_per_unit", .0006)
            if assembled:
                self.stl_display_meshes.append((prim, points.copy()))
                prim.GetPrim().SetCustomDataByKey("deformation", "DISPLAY ONLY: height-weighted FEM frame blend; no solid stress")
            return prim

        assembled = UsdGeom.Xform.Define(stage, root_path + "/Assembled")
        assembled.GetPrim().SetCustomDataByKey("soft_source", assets["soft"].sha256)
        assembled.GetPrim().SetCustomDataByKey("assembled_source", assets["assembled"].sha256)
        add_triangles(root_path + "/Assembled/SoftPET", assets["soft"].triangles, soft_mat, "assembled flexible shell (Soft.stl)")
        add_triangles(root_path + "/Assembled/RigidPLA", assets["assembled_rigid_triangles"], rigid_mat,
                      "assembled rigid plates (soft and rigid sections.stl minus Soft.stl)")

        layout = UsdGeom.Xform.Define(stage, root_path + "/FlatRigidLayout")
        layout.GetPrim().SetCustomDataByKey("source", assets["rigid_layout"].sha256)
        # Keep the manufacturing sheet out of the FEM overlay by default.  It
        # is placed to the side when enabled so its bed coordinates are not
        # mistaken for the assembled joint frame.
        translate_ops = [op for op in layout.GetOrderedXformOps()
                         if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
        (translate_ops[0] if translate_ops else layout.AddTranslateOp()).Set(Gf.Vec3d(-.075, -.045, 0.0))
        add_triangles(root_path + "/FlatRigidLayout/RigidPLA", assets["rigid_layout"].triangles,
                      layout_mat, "flat rigid manufacturing layout (Rigid.stl)", assembled=False)
        self.stl_reference = assembled
        self.stl_layout = layout
        UsdGeom.Imageable(assembled.GetPrim()).MakeInvisible()
        UsdGeom.Imageable(layout.GetPrim()).MakeInvisible()
        if self.stl_reference_visible:
            UsdGeom.Imageable(assembled.GetPrim()).MakeVisible()
        if self.stl_layout_visible:
            UsdGeom.Imageable(layout.GetPrim()).MakeVisible()
        if hasattr(self, "stl_status"):
            self.stl_status.text = ("STL reference loaded | assembled "
                                     + ("visible" if self.stl_reference_visible else "hidden")
                                     + " | flat Rigid "
                                     + ("visible" if self.stl_layout_visible else "hidden"))

    def toggle_stl_reference(self):
        if self.stl_reference is None:
            return
        self.stl_reference_visible = not self.stl_reference_visible
        imageable = UsdGeom.Imageable(self.stl_reference.GetPrim())
        (imageable.MakeVisible if self.stl_reference_visible else imageable.MakeInvisible)()
        self.stl_status.text = ("STL reference loaded | assembled "
                                 + ("visible" if self.stl_reference_visible else "hidden")
                                 + " | flat Rigid "
                                 + ("visible" if self.stl_layout_visible else "hidden"))
        self.render()

    def toggle_stl_layout(self):
        if self.stl_layout is None:
            return
        self.stl_layout_visible = not self.stl_layout_visible
        imageable = UsdGeom.Imageable(self.stl_layout.GetPrim())
        (imageable.MakeVisible if self.stl_layout_visible else imageable.MakeInvisible)()
        self.stl_status.text = ("STL reference loaded | assembled "
                                 + ("visible" if self.stl_reference_visible else "hidden")
                                 + " | flat Rigid "
                                 + ("visible" if self.stl_layout_visible else "hidden"))
        self.render()

    def set_time(self, elapsed):
        self.elapsed = elapsed  # Base initialization only; our jobs own the release.

    def whole_view(self):
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[.34, -.51, .3], target=[0, 0, .125])

    def close_view(self):
        center = self.offset+self.shell.height/2 if self.shell else .132
        if self.stl_reference_visible:
            # Keep the lower CAD display in shot when Apply starts a solve.
            target_x = .028 if self.show_response else 0
            ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[.15, -.29, center+.09],
                                            target=[target_x, 0, (center+self.shell.height/2)/2])
        elif self.show_response:
            center = self.lab.scene.top_height-self.shell.height/2
            ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[.090, -.175, center+.060], target=[.028, 0, center])
        else:
            ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[.08, -.13, center+.045], target=[0, 0, center])

    def change_presentation(self, smooth=None, strain=None):
        if smooth is not None:
            self.smooth_motion = bool(smooth)
        if strain is not None:
            self.strain_colours = bool(strain)
        if self.ensure_current_scene():
            self.render()

    def begin_transition(self, previous, duration=.8):
        if not self.smooth_motion:
            self.render()
            return
        self.accepted_report = self.shell.diagnostics(self.state)
        self.presentation = SolvedTransition(previous, self.offset)
        self.presentation.begin(previous, self.state, duration, self.offset, self.offset)
        self.presentation_clock = time.perf_counter()
        self.render(display_state=previous)

    async def finish_transition(self):
        while self.presentation is not None and self.presentation.active:
            await self.wait_running()
            await omni.kit.app.get_app().next_update_async()

    def render(self, display_state=None):
        if self.owned is None:
            return
        interpolated = display_state is not None
        if not interpolated:
            self.presentation = SolvedTransition(self.state, self.offset)
            self.accepted_report = self.shell.diagnostics(self.state)
        report = self.accepted_report
        shown_state = np.asarray(display_state) if interpolated else self.state
        shown_points = self.shell.positions(self.shell._tensor(shown_state)).numpy() if interpolated else report["points"]
        if self.stl_reference_visible:
            frame_states = shown_state[self.shell.frame_start:self.shell.frame_start+12].reshape(2, 6)
            gain = self.response_gain if self.show_response else 1.0
            for cad_mesh, neutral_points in getattr(self, "stl_display_meshes", []):
                cad_points, _ = frame_blend_display(neutral_points, self.shell.frame_centers.numpy(),
                    self.shell.width*frame_states[:, :3], Rotation.from_rotvec(frame_states[:, 3:]).as_matrix(), gain)
                set_points(cad_mesh, cad_points)
            self.stl_status.text = (f"LOWER STL: frame-motion display x{gain:g}, not solid FEM. "
                                   "Upper joint/values = JSON shell FEM. Flat layout stays static.")
        points = shown_points + [0, 0, self.offset]
        set_points(self.surface, points)
        values = np.clip(report["principal_membrane_strain"]/self.shell.config.panel_strain_limit, 0, 1)
        colors = np.stack([.12+.88*values, .45+.25*(1-np.abs(2*values-1)), .95*(1-values)], axis=1)
        if not self.strain_colours or interpolated:
            colors = np.where(self.shell.mesh["laminate"][:, None], [.88, .89, .80], [.35, .60, .64])
        self.presentation_label.text = ("DISPLAY TRANSITION: strain colours hidden; numbers are the accepted endpoint, NOT this intermediate pose."
                                        if interpolated else "ACCEPTED FRAME: calculated geometry and results. White = PLA/PET; blue-green = PET only.")
        self.surface.GetDisplayColorAttr().Set(Vt.Vec3fArray.FromNumpy(colors.astype(np.float32)))
        set_points(self.lines, points[self.line_edges].reshape(-1, 3))
        if self.relief_lines is not None:
            set_points(self.relief_lines, points[self.relief_edges].reshape(-1, 3))
        if getattr(self, "frame_hinge_lines", None) is not None:
            set_points(self.frame_hinge_lines, points[self.frame_hinge_edges].reshape(-1, 3))
        cfg = self.shell.config
        self.settings_label.text = (f"ACTIVE: panel bend x{cfg.panel_bending_scale:g}, membrane x{cfg.membrane_scale:g}, "
            + (f"PET fold compliance {cfg.panel_to_crease_ratio:g} (100 = PET reference)" if cfg.physical_strip_bending
               else f"panel/fold ratio {cfg.panel_to_crease_ratio:g}")
            + f", gap {self.shell.material.hinge_gap_m*1000:g} mm\n"
            f"PLA {self.shell.material.pla_thickness_m*1000:g} mm + PET {self.shell.material.pet_thickness_m*1e6:g} um; "
            f"mesh {2**cfg.subdivision} edge segments / interior {cfg.interior_refinement}, " + ("IPC midsurface CCD" if cfg.self_contact else "NO self-contact forces") + "\n"
            f"Frame hinges {cfg.frame_hinge_width_m*1000:g} mm; PET junction relief {cfg.vertex_relief_fraction*100:g}% "
            + ("- MODIFIED CUT DESIGN" if cfg.vertex_relief_fraction else ("- PHOTO OPEN ENDS; JSON side panels" if cfg.open_ends else "- JSON capped comparison")))
        set_points(self.frames_curve, points[self.frame_edges].reshape(-1, 3))
        state_tensor = self.shell._tensor(shown_state)
        top = self.shell.frame_point(state_tensor, 0, self.shell.cable_top).numpy()+[0, 0, self.offset]
        bottom = self.shell.frame_point(state_tensor, 1, self.shell.cable_bottom).numpy()+[0, 0, self.offset]
        cable = np.stack([top, bottom], axis=1)
        set_points(self.axial, cable[:4].reshape(-1, 3))
        set_points(self.red, cable[4:].reshape(-1, 3))
        set_points(self.loaded_cables, cable.reshape(-1, 3))
        loaded = self.active_cable_mask | (self.tensions > 1e-10)
        self.loaded_cables.GetWidthsAttr().Set((.00032*loaded).tolist())
        if np.any(loaded):
            self.loaded_cables.MakeVisible()
        else:
            self.loaded_cables.MakeInvisible()
        if self.static_trace and self.drop is None:
            nodal_loads = np.asarray(self.static_trace[self.display_index if self.display_index is not None else -1]["nodal_loads_n"])
            applied = nodal_loads.sum(axis=0)
            weights = np.linalg.norm(nodal_loads, axis=1)
            center = (np.average(points, axis=0, weights=weights) if weights.sum() > 0
                      else points[self.shell.mesh["bottom"]].mean(axis=0))
            set_points(self.applied_force, np.stack([center, center+.002*applied]))
            self.applied_force.MakeVisible() if np.linalg.norm(applied) > 1e-10 else self.applied_force.MakeInvisible()
        else:
            self.applied_force.MakeInvisible()
        corners = top[:4].copy()
        set_points(self.top_cross, corners[[0, 2, 1, 3]])
        roof_warp = []
        for frame, op in enumerate(self.body_ops):
            q = shown_state[self.shell.frame_start+6*frame:self.shell.frame_start+6*(frame+1)]
            rotation = Rotation.from_rotvec(q[3:]).as_matrix()
            center = self.shell.frame_centers[frame].numpy()+self.shell.width*q[:3]+[0, 0, self.offset]
            matrix = np.eye(4)
            matrix[:3, :3], matrix[3, :3] = rotation.T, center
            op.Set(Gf.Matrix4d(*matrix.ravel().tolist()))
            mask = np.isclose(self.shell.points[:, 2], self.shell.frame_centers[frame, 2].item(), atol=1e-12)
            accepted_q = self.state[self.shell.frame_start+6*frame:self.shell.frame_start+6*(frame+1)]
            accepted_rotation = Rotation.from_rotvec(accepted_q[3:]).as_matrix()
            accepted_center = self.shell.frame_centers[frame].numpy()+self.shell.width*accepted_q[:3]
            roof_warp.append(float(np.abs((report["points"][mask]-accepted_center) @ accepted_rotation[:, 2]).max()))
        row = self.drop.trace[self.display_index if self.display_index is not None else -1] if self.drop else None
        force = row["ground_force_n"] if row else 0
        if self.drop and force > 0:
            foot = self.shell.frame_point(state_tensor, 1, self.drop.foot_points).numpy()+[0, 0, self.offset]
            loads = np.asarray(row["corner_forces_n"])
            arrows = np.stack([foot, foot+np.c_[np.zeros((4, 2)), .002*loads]], axis=1)
            set_points(self.forces, arrows.reshape(-1, 3))
            self.forces.MakeVisible()
        else:
            self.forces.MakeInvisible()
        angles = np.rad2deg(report["relative_rotation_rad"])
        clock = row["time_after_contact_s"]*1000 if row else 0
        self.telemetry.text = (f"{self.mode} | ACCEPTED RESULT (1x)\n"
            f"Bend X/Y {angles[0]:.3f} / {angles[1]:.3f} deg | twist {angles[2]:.3f} deg\n"
            f"Compression {report['compression_fraction']*100:.3f}% | membrane strain {report['max_membrane_strain']*100:.3f}%\n"
            + ("Open ends; no roof plates or roof force test\n" if cfg.open_ends else f"Upper/lower roof warp {roof_warp[0]*1e6:.2f} / {roof_warp[1]*1e6:.2f} um\n")
            + f"Frame PET local hinge rotation {np.rad2deg(report['frame_hinge']['max_local_dihedral_change_rad']):.3f} deg\n"
            f"Reaction {np.linalg.norm(self.last_reaction[:3]):.4g} N / {np.linalg.norm(self.last_reaction[3:]):.4g} N m | "
            f"Ground force {force:.3f} N | contact time +{clock:.3f} ms")
        self.legend.text = (f"{self.mode.split(';')[0]} | RIGID GOLD FRAMES | 1x\n"
                            f"Bend Y {angles[1]:.3f} deg | twist {angles[2]:.3f} deg | compression {report['compression_fraction']*100:.2f}%\n"
                            "Uncalibrated shell | strain, NOT failure | Ground lines: 2 mm/N")
        if self.inspector is not None:
            self.inspector.update(shown_state)
            if self.show_response:
                self.legend.text = (f"LEFT: physical leg 1x | RIGHT: displacement x{self.response_gain:g}, DISPLAY ONLY\n"
                    f"TRUE bend Y {angles[1]:.3f} deg | twist {angles[2]:.3f} deg | compression {report['compression_fraction']*100:.3f}%\n"
                    "Right is a vector plot, NOT a physical fold or validated capacity.")
        if interpolated:
            self.legend.text = "DISPLAY TRANSITION - NOT A SOLVED FEM STATE\nAccepted endpoint results shown in controls; strain colours hidden.\nNo physical time or equilibrium is assigned to this transition."
            self.presentation_frames += 1
        self.frames += 1

    def neutral_shell(self):
        self.cancel_jobs()
        if not self.ensure_current_scene():
            return
        if self.inspector is not None:
            self.inspector.playing = False
        self.state = np.zeros(self.shell.ndof)
        self.drop = None
        self.display_index = None
        self.tensions = np.zeros(12)
        self.active_cable_mask = np.zeros(12, dtype=bool)
        self.last_reaction = np.zeros(6)
        self.active_family = None
        self.offset = self.lab.scene.top_height-self.shell.height
        self.mode = "NEUTRAL"
        self.snapshots = []
        self.static_trace = []
        self.last_rejection = None
        self.render()
        self.feedback.text = "Neutral. Source side-panel geometry restored; PET connections remain flexible."

    def cancel_jobs(self):
        super().cancel_jobs()
        self.presentation = None

    def pause_jobs(self):
        if (self.inspector is not None and self.inspector.playing
                and (self.work is None or self.work.done())):
            self.inspector.toggle_pause()
        else:
            super().pause_jobs()

    async def rebuild(self):
        get = lambda name, default: self.inputs[name].as_float if name in self.inputs else default
        old = self.shell.config
        config = dataclasses.replace(old,
                             panel_to_crease_ratio=get("Fold compliance (100 = PET reference)", old.panel_to_crease_ratio),
                             panel_bending_scale=get("Panel bending scale", old.panel_bending_scale),
                             membrane_scale=get("Membrane stiffness scale", old.membrane_scale),
                             frame_hinge_width_m=get("PET hinge width (mm)", old.frame_hinge_width_m*1000)/1000,
                             max_iterations=900, sparse_solver=True, physical_strip_bending=True,
                             self_contact=True, open_ends=True, interior_refinement=0, subdivision=1)
        config.validate()
        config = dataclasses.replace(config, max_iterations=int(config.max_iterations))
        self.feedback.text = "Assembling the new nonlinear shell; displayed result held until ready."
        material = dataclasses.replace(self.shell.material,
            pla_modulus_pa=get("PLA modulus (GPa)", self.shell.material.pla_modulus_pa/1e9)*1e9,
            pet_modulus_pa=get("PET modulus (GPa)", self.shell.material.pet_modulus_pa/1e9)*1e9)
        material.validate()
        candidate = await self.compute(NonlinearShell, self.lab.model.source, material, config)
        self.shell = candidate
        self.state = np.zeros(candidate.ndof)
        self.drop = None
        self.display_index = None
        self.snapshots = []
        self.tensions = np.zeros(12)
        self.active_cable_mask = np.zeros(12, dtype=bool)
        self.last_reaction = np.zeros(6)
        self.active_family = None
        self.last_rejection = None
        self.mode = "NEUTRAL - NEW STIFFNESS"
        self.static_trace = []
        self.offset = self.lab.scene.top_height-candidate.height
        # Topology changes need new USD faces, curves and cached schema handles.
        self.stage.RemovePrim(self.owned_path)
        self.owned = UsdGeom.Xform.Define(self.stage, self.owned_path)
        self.owned.GetPrim().SetCustomDataByKey("source_sha256", candidate.source["sha256"])
        self.owned.GetPrim().SetCustomDataByKey("scope", "Experimental shell; junction relief changes the manufacturing pattern")
        self.make_scene()
        self.refresh_pose_controls()
        self.render()
        self.feedback.text = "Stiffness rebuilt. New controls apply to subsequent range/cable/drop calculations."

    async def preset(self, relief, frame_hinges=False, open_ends=False):
        values = {"Fold compliance (100 = PET reference)": 1000 if relief else 100,
                  "Crease twist coupling": .1 if relief else 0, "Panel bending scale": .01 if relief else 1,
                  "PLA thickness (mm)": .4, "PET thickness (um)": 80,
                  "PLA modulus (GPa, assumed)": 2.2, "PET modulus (GPa, assumed)": 3.5,
                  "Membrane stiffness scale": 1, "PET junction relief (%)": 20 if relief else 0,
                  "PET exposed gap (mm)": .8 if relief else .2, "Membrane strain guard (%)": 3,
                  "Frame-plate PET hinge (mm)": .2 if (frame_hinges or open_ends) else 0,
                  "Static solver iterations": 3000 if relief else 900,
                  "Winch pull (mm)": .5, "Winch stiffness (N/m)": 1000, "Winch force cap (N)": 10}
        for name, value in values.items():
            self.inputs[name].set_value(value)
        self.flags["Sparse element solver"].set_value(True)
        self.flags["Thickness-derived PET bending (100 = PET reference)"].set_value(not relief)
        self.flags["Midsurface self-contact + CCD"].set_value(True)
        self.flags["Open ends (photo design, no JSON caps)"].set_value(open_ends)
        self.refinement_input.set_value(0 if (relief or open_ends) else 1)
        self.boundary_input.set_value(1)
        await self.rebuild()
        self.feedback.text = ("Experimental PET cuts + 100x softer bending; NOT measured PLA/PET. "
                              "Use Apply winch pull. Adjacent-section twist coupling is inactive in this coarse cut mesh." if relief
                              else "Intact PLA 0.4 mm / PET 80 um; unscaled assumed moduli, thickness-derived bending. "
                                   "Refined mesh + midsurface IPC; NOT calibrated creased-PET or finite-thickness contact.")
        if frame_hinges:
            self.feedback.text = ("Both roof plates now connect to their rigid frames through 0.2 mm PET flexure borders (green). "
                                  "Actual thicknesses, assumed moduli; movement is solved from force, not an imposed angle.")
        if open_ends:
            self.feedback.text = ("PHOTO DESIGN: open square ends, unchanged JSON side panels, continuous PET frame hinges. "
                                  "Mounting bands are schematic; lug/hole dimensions and material response need measurement. No cuts or artificial softening.")

    def start_drop(self):
        self.timeline_action = "drop"
        self.launch(self.drop_job(), "70 mm impact")

    async def drop_job(self):
        self.inspector.playing = False
        get = lambda name: self.inputs[name].as_float
        config = ShellImpactConfig(upper_mass_kg=get("Upper-side mass (g)")/1000,
            lower_mass_kg=get("Lower-side mass (g)")/1000, contact_stiffness_n_m=get("Contact stiffness (N/m)"),
            contact_damping_ns_m=get("Contact damping (N s/m)"), step_s=get("Drop timestep (us)")*1e-6,
            duration_s=get("Drop duration (ms)")*.001)
        candidate = await self.compute(ShellImpact, self.shell, config)
        self.drop, self.state = candidate, candidate.state.copy()
        self.job_steps = config.step_count
        self.display_index = None
        self.snapshots, self.last_rejection = [], None
        self.static_trace = []
        self.tensions = np.zeros(12)
        self.mode = "FREE FALL; CABLES UNLOADED"
        elapsed, previous = 0.0, time.perf_counter()
        contact_time = math.sqrt(2*config.height_m/9.81)
        self.whole_view()
        while elapsed < contact_time:
            await self.wait_running()
            await omni.kit.app.get_app().next_update_async()
            now = time.perf_counter()
            elapsed = min(contact_time, elapsed+min(now-previous, .05)*.05)
            previous = now
            self.offset = .0825+config.height_m-.5*9.81*elapsed**2
            self.render()
        self.offset = .0825
        self.mode = "LIVE NONLINEAR IMPACT"
        self.close_view()
        while not self.drop.stopped:
            await self.wait_running()
            self.feedback.text = "Solving nodal/frame inertia + shell deformation + foot contact..."
            candidate = copy.copy(self.drop)
            candidate.trace = self.drop.trace.copy()
            await self.compute(candidate.step)
            await self.wait_running()
            await self.finish_transition()
            advanced = candidate.time_s > self.drop.time_s
            previous_state = self.state.copy()
            self.drop = candidate
            self.state = candidate.state.copy()
            if advanced:
                self.job_step += 1
                self.snapshots.append(self.state.copy())
            if advanced:
                self.begin_transition(previous_state, .25)
            else:
                self.render()
            await omni.kit.app.get_app().next_update_async()
        self.last_rejection = self.drop.last_candidate if "GUARD" in str(self.drop.reason) or "LIMIT" in str(self.drop.reason) else None
        if self.last_rejection is not None:
            self.job_outcome = "STOPPED: numerical guard"
        await self.finish_transition()
        self.feedback.text = self.drop.reason+"\nThis is an uncalibrated mechanical response, not a survival verdict."

    def start_range(self, mode):
        self.launch(self.range_job(mode), mode.replace("_", " "))

    async def range_job(self, mode, family=None, amplitude=None):
        self.inspector.playing = False
        jobs = []
        if mode == "displacement":
            raise ValueError("Displacement control was retired for the photo/video joint. Use the cable-family pull so the original fold pattern is preserved.")
        elif mode in ("bend", "twist", "compression"):
            if self.shell.contact is not None:
                raise ValueError("Prescribed-frame studies do not support IPC yet; use force/winch actuation, "
                                 "or explicitly disable self-contact and Rebuild for a non-contact displacement study")
            label = {"bend": "Bend target (deg)", "twist": "Twist target (deg)", "compression": "Compression target (%)"}[mode]
            target = self.inputs[label].as_float
            if not np.isfinite(target) or not 0 < target <= (90 if mode == "compression" else 150):
                raise ValueError("Use a positive target up to 90% compression or 150 degrees")
            steps = max(1, math.ceil(target/2))
            for value in np.linspace(0, target, steps+1)[1:]:
                coordinate = {"bend": 4, "twist": 5, "compression": 2}[mode]
                si_value = self.shell.height*value/100 if mode == "compression" else np.deg2rad(value)
                jobs.append(({"lower_constraints": {coordinate: si_value}}, f"PRESCRIBED {mode.upper()} {value:.1f}; other coordinates free"))
        elif mode in ("cables", "manual", "single_cable", "winch", "winch_demo"):
            if amplitude is None:
                amplitude = self.inputs["Cable tension (N)"].as_float
            if mode in ("cables", "manual", "single_cable") and (not np.isfinite(amplitude) or not 0 <= amplitude <= 10):
                raise ValueError("Experimental cable tension must be 0-10 N")
            families = CABLE_FAMILIES
            force = np.zeros(3)
            if mode in ("manual", "winch", "single_cable"):
                if family is None:
                    index = self.cable_pattern.model.get_item_value_model().as_int
                    family = families[index]
                if family not in families:
                    raise ValueError("Unknown cable movement")
                families = (family,)
                if mode != "single_cable":
                    force = np.array([self.inputs[f"Lower frame force {axis} (N)"].as_float for axis in "XYZ"])
                if not np.isfinite(force).all() or np.abs(force).max() > 100:
                    raise ValueError("Use finite experimental frame-force components within +/-100 N")
            nodes = self.shell.mesh["bottom"]
            for family in families:
                # Accepted load steps expose the response even when the same
                # endpoint is requested twice; a one-step repeat looked inert.
                fractions = (.25, .5, .75, 1)
                if mode in ("winch", "winch_demo"):
                    pull = self.inputs["Winch pull (mm)"].as_float
                    if not np.isfinite(pull) or not 0 <= pull <= 20:
                        raise ValueError("Winch pull must be 0-20 mm; this is cable take-up, not a joint pose")
                    fractions = np.linspace(0, 1, max(2, math.ceil(pull/.25))+1)[1:].tolist()
                    if mode == "winch_demo":
                        fractions += fractions[-2::-1]+[0]
                for fraction in fractions:
                    loads = np.zeros_like(self.shell.points)
                    loads[nodes] = fraction*force/len(nodes)
                    if mode in ("winch", "winch_demo"):
                        pull = self.inputs["Winch pull (mm)"].as_float
                        if not np.isfinite(pull) or not 0 <= pull <= 20:
                            raise ValueError("Winch pull must be 0-20 mm; this is cable take-up, not a joint pose")
                        winch = winch_pull(self.shell, family, fraction*pull/1000,
                                          self.inputs["Winch stiffness (N/m)"].as_float,
                                          self.inputs["Winch force cap (N)"].as_float)
                        jobs.append(({"winch": winch, "nodal_loads": loads}, f"ELASTIC WINCH: {family}, pull {fraction*pull:g} mm"))
                    else:
                        jobs.append(({"tensions": tension_pattern(family, amplitude*fraction), "nodal_loads": loads},
                                     "FORCE-DRIVEN: "+family))
        else:
            if self.shell.config.open_ends:
                raise ValueError("Photo design has no roof caps. Use cable actuation or lower-frame force instead.")
            amplitude = self.inputs["Roof test force (N)"].as_float
            if not np.isfinite(amplitude) or not 0 <= amplitude <= 10:
                raise ValueError("Experimental roof force must be 0-10 N")
            nodes = self.shell.free_nodes[np.isclose(self.shell.points[self.shell.free_nodes, 2], self.shell.height)]
            center = nodes[np.argmin(np.linalg.norm(self.shell.points[nodes, :2], axis=1))]
            load = np.zeros_like(self.shell.points)
            load[center, 2] = -amplitude
            jobs.append(({"nodal_loads": load}, f"ROOF CENTER FORCE {amplitude:g} N; PERIMETER FIXED"))
        # Invalid inputs must leave the last accepted physical state intact.
        self.timeline_action = mode
        initial = np.zeros(self.shell.ndof)
        self.close_view()
        self.job_steps, self.job_step = len(jobs), 0
        for index, (parameters, label) in enumerate(jobs):
            await self.wait_running()
            self.feedback.text = "Solving "+label+"; last accepted shape remains visible."
            solve_started = time.perf_counter()
            state, report = await self.compute(self.shell.solve, initial=initial, **parameters)
            solve_wall_s = time.perf_counter()-solve_started
            await self.wait_running()
            await self.finish_transition()
            if not report["accepted"]:
                self.job_outcome = "STOPPED: numerical guard"
                self.last_rejection = report
                self.feedback.text = (f"STOPPED: candidate NOT applied. Residual {report['gradient_max_j_per_scaled_coordinate']:.3g}; "
                    f"strain {report['max_membrane_strain']*100:.2f}%; intersections {len(report['surface_intersection_pairs'])}.\n"
                    "Requested full range has NOT been established. Last accepted state held.")
                return
            if index == 0:
                self.drop, self.last_rejection = None, None
                self.display_index = None
                self.offset = self.lab.scene.top_height-self.shell.height
                self.snapshots, self.static_trace = [], []
            previous_state = self.state.copy()
            self.state = initial = state
            self.mode = label
            self.job_step = index+1
            self.tensions = np.asarray(report["tensions_n"])
            if mode == "displacement":
                self.active_cable_mask = tension_pattern(self.active_family, 1.0) > 0
            else:
                self.active_cable_mask = self.tensions > 1e-10
            self.last_reaction = np.asarray(report["required_frame_reactions_world_n_nm"][1], dtype=float)
            self.snapshots.append(state.copy())
            self.static_trace.append({"mode": label, "tensions_n": self.tensions.tolist(),
                                      "solve_wall_s": solve_wall_s,
                                      "winch": report.get("winch"), "cable_lengths_m": report.get("cable_lengths_m"),
                                      "self_contact": report.get("self_contact"),
                                      "nodal_loads_n": np.asarray(parameters.get("nodal_loads", np.zeros_like(self.shell.points))).tolist(),
                                      "relative_rotation_rad": report["relative_rotation_rad"].tolist(),
                                      "compression_fraction": report["compression_fraction"],
                                      "max_membrane_strain": report["max_membrane_strain"],
                                      "frame_hinge": report["frame_hinge"],
                                      "reaction_world_n_nm": self.last_reaction.tolist(),
                                      "residual": report["gradient_max_j_per_scaled_coordinate"]})
            # Present only accepted endpoints; the next solve runs concurrently.
            # This easing is display-time, NOT force-balanced dynamics.
            # Keep the accepted endpoint, but give the viewer a stable smooth
            # transition even when the coarse side-only solve finishes quickly.
            self.begin_transition(previous_state, min(3.0, max(1.5, solve_wall_s*.8)))
            reaction = self.last_reaction
            if mode == "displacement":
                self.feedback.text = (f"FEM displacement {family}: {label.split()[-2]} mm accepted. "
                    f"Lower-frame reaction {np.linalg.norm(reaction[:3]):.4g} N / "
                    f"{np.linalg.norm(reaction[3:]):.4g} N m.\n"
                    "The displayed cable is the prescribed actuator direction; force is a solved reaction.")
            elif mode in ("manual", "single_cable", "cables", "roof", "winch", "winch_demo"):
                angles = np.rad2deg(report["relative_rotation_rad"])
                self.feedback.text = (f"Converged. Cable peak {max(self.tensions):.3g} N/strand. "
                    f"Bend X/Y {angles[0]:.3f}/{angles[1]:.3f} deg, twist {angles[2]:.3f} deg, "
                    f"compression {report['compression_fraction']*100:.3f}%.\n"
                    f"Residual {report['gradient_max_j_per_scaled_coordinate']:.2g} J/scaled coordinate; "
                    "shape and strain are solved at these loads, no angle is imposed.")
            else:
                self.feedback.text = (f"Prescribed study. Required moment X/Y/Z: "
                    f"{reaction[3]:.4g} / {reaction[4]:.4g} / {reaction[5]:.4g} N m.")
            await omni.kit.app.get_app().next_update_async()
        await self.finish_transition()
        final_report = self.shell.diagnostics(self.state)
        final_angles = np.rad2deg(final_report["relative_rotation_rad"])
        final_reaction = np.asarray(self.last_reaction, dtype=float)
        if mode == "displacement":
            self.feedback.text = (f"COMPLETE: {self.job_step}/{self.job_steps} displacement FEM steps. "
                                  f"Bend X/Y {final_angles[0]:.3f}/{final_angles[1]:.3f} deg, "
                                  f"twist {final_angles[2]:.3f} deg, compression {final_report['compression_fraction']*100:.3f}%. "
                                  f"Reaction {np.linalg.norm(final_reaction[:3]):.4g} N / {np.linalg.norm(final_reaction[3:]):.4g} N m. "
                                  "Magnification is display-only; full physical travel is not yet calibrated.")
        else:
            self.feedback.text = (f"COMPLETE: {self.job_step}/{self.job_steps} cable FEM step(s) using the original photo/video pattern. "
                                  f"Bend X/Y {final_angles[0]:.3f}/{final_angles[1]:.3f} deg, "
                                  f"twist {final_angles[2]:.3f} deg, compression {final_report['compression_fraction']*100:.3f}%. "
                                  f"Reaction {np.linalg.norm(final_reaction[:3]):.4g} N / {np.linalg.norm(final_reaction[3:]):.4g} N m. "
                                  "The cable load is solved; magnification is display-only and full travel is not calibrated.")

    def show_peak(self):
        if not self.ensure_current_scene():
            return
        if self.work and not self.work.done():
            self.feedback.text = "Wait for the current calculation before inspecting recorded frames."
            return
        if self.drop is None or not self.snapshots:
            self.feedback.text = "Run a drop first to record force-matched frames."
            return
        self.display_index = int(np.argmax([np.linalg.norm(row["relative_rotation_rad"][:2]) for row in self.drop.trace]))
        self.state = self.snapshots[self.display_index-1].copy() if self.display_index else np.zeros(self.shell.ndof)
        self.mode = "RECORDED PEAK BENDING"
        self.render()
        self.close_view()
        self.feedback.text = "Inspecting the solved peak-bending frame with its matching time and contact force. No new solve."

    async def replay_recorded(self):
        if not self.snapshots:
            raise ValueError("Run cable actuation or a drop first; no solved states are recorded")
        self.mode = "RECORDED SOLVED-STATE REPLAY (DISPLAY TIME)"
        self.close_view()
        final_state, final_tensions = self.state.copy(), self.tensions.copy()
        final_index = self.display_index
        try:
            for index, state in enumerate(self.snapshots):
                await self.wait_running()
                previous_state = self.state.copy()
                self.state = state.copy()
                self.display_index = index+1 if self.drop is not None else index
                if self.drop is None and self.static_trace:
                    self.tensions = np.asarray(self.static_trace[index]["tensions_n"])
                self.begin_transition(previous_state, .5)
                await self.finish_transition()
        finally:
            self.state, self.tensions = final_state, final_tensions
            self.display_index = final_index
            if self.scene_is_current():
                self.render()
        self.feedback.text = "Smooth replay finished. Only endpoints are FEM solutions; transitions are display-only. Final accepted result restored."

    async def loop(self):
        try:
            while not self.closed:
                await omni.kit.app.get_app().next_update_async()
                self.progress_label.text = self.progress_text()
                if not self.scene_is_current():
                    if not self.scene_disconnected:
                        self.cancel_jobs()
                        self.scene_disconnected = True
                        self.job_outcome = "STOPPED: scene changed"
                        self.feedback.text = "Scene changed. Click Reconnect opened knee to bind this workshop to the matching saved USD."
                    continue
                if self.inspector is not None:
                    self.inspector.tick()
                now = time.perf_counter()
                if self.presentation is not None and self.presentation.active and now-self.presentation_clock >= 1/30:
                    dt = now-self.presentation_clock
                    self.presentation_clock = now
                    shown_state, _ = self.presentation.advance(dt, paused=not self.running)
                    if self.running:
                        self.render(display_state=shown_state) if self.presentation.active else self.render()
                playing = app_utils.is_playing()
                if playing != self.timeline_was_playing:
                    self.timeline_was_playing = playing
                    if playing and (self.work is None or self.work.done()):
                        if self.timeline_action == "drop":
                            self.start_drop()
                        else:
                            self.start_range(self.timeline_action)
                    else:
                        self.running = playing
        except asyncio.CancelledError:
            pass

    def save(self):
        if not self.ensure_current_scene():
            return None
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        folder = self.lab.project/"exact_joint/results"/(stamp+"_nonlinear_shell")
        folder.mkdir(parents=True, exist_ok=False)
        convert = lambda value: value.tolist() if isinstance(value, np.ndarray) else str(value)
        report = {"source_sha256": self.shell.source["sha256"], "material": dataclasses.asdict(self.shell.material),
                  "shell_config": dataclasses.asdict(self.shell.config), "mode": self.mode,
                  "display": {"physical_geometry_gain": 1, "diagnostic_gain": self.response_gain,
                              "diagnostic_visible": self.show_response,
                              "smooth_transitions": self.smooth_motion,
                              "intermediate_states_saved": False,
                              "transition_scope": "Display-only easing of accepted endpoints; no intermediate stresses or physical times",
                              "scope": "Diagnostic vector plot only; not a physical configuration or strain magnification"},
                  "inspected_trace_index": self.display_index,
                  "tensions_n": self.tensions.tolist(), "static_trace": self.static_trace,
                  "state": self.state, "snapshots": self.snapshots, "current": self.shell.diagnostics(self.state),
                  "last_rejected_candidate": self.last_rejection, "survives": None}
        if self.shell.contact is not None:
            report["self_contact"] = self.shell.contact.report(report["current"]["points"])
        if self.drop:
            report["impact_config"] = dataclasses.asdict(self.drop.config)
            report["impact_trace"] = self.drop.trace
            report["shell_mass_kg"] = self.drop.shell_mass_kg
        (folder/"calculation.json").write_text(json.dumps(report, indent=2, default=convert), encoding="utf-8")
        self.saved_path = folder
        self.feedback.text = "Saved computed state, trace and assumptions: "+folder.name
        return folder

    async def restore(self):
        if self.closed:
            return
        previous_work = self.work
        self.cancel_jobs()
        if previous_work:
            try:
                await previous_work
            except asyncio.CancelledError:
                pass
        if self.owned and self.owned.GetPrim().IsValid():
            self.stage.RemovePrim(self.owned_path)
        if self.root.IsValid():
            self.root.SetActive(getattr(self, "original_leg_active", True))
            # Reactivation recreates descendant prims; the old schema handles
            # are invalid even though the paths exist again. Rebind, do not
            # rebuild or replace the user's original stage/model.
            if self.stage == omni.usd.get_context().get_stage():
                self.lab.scene = ExactLegScene.attach(self.stage, self.lab.model)
        await super().restore()
