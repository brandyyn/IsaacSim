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
from exact_joint.mechanics import tension_pattern
from exact_joint.nonlinear_shell import NonlinearShell, ShellConfig
from exact_joint.scene import ExactLegScene, curves, material, mesh, set_points
from exact_joint.shell_impact import ShellImpact, ShellImpactConfig


class ShellWorkshop(DropPreview):
    def __init__(self, lab):
        super().__init__(lab)
        self.shell = None
        self.work = None
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

    async def start(self):
        self.shell = await asyncio.to_thread(NonlinearShell, self.lab.model.source, self.lab.config)
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
        super().build_ui()
        self.window.title = "Nonlinear knee - rigid frames / flexible panels"
        with self.window.frame, ui.VStack(spacing=5):
            ui.Label("RIGID SQUARE FRAMES | FLEXIBLE INTERIORS", height=26, style={"font_size": 18})
            ui.Label("NONLINEAR SHELL EXPERIMENT - not validated solid FEM\n1x geometry; colour = membrane strain, NOT failure/stress", height=43,
                     style={"color": 0xFF80DFFF})
            with ui.HStack(height=29):
                ui.Button("Drop 70 mm / recompute", clicked_fn=self.start_drop)
                ui.Button("Pause / resume", clicked_fn=lambda: setattr(self, "running", not self.running))
                ui.Button("Neutral / cancel", clicked_fn=self.neutral_shell)
            with ui.HStack(height=29):
                self.cable_pattern = ui.ComboBox(0, "Compression", "Bend X+", "Bend X-", "Bend Y+", "Bend Y-", "Twist CW", "Twist CCW")
                ui.Button("Apply cable + force", clicked_fn=lambda: self.start_range("manual"))
            with ui.HStack(height=29):
                ui.Button("Cable demo", clicked_fn=lambda: self.start_range("cables"))
                ui.Button("Roof flex test", clicked_fn=lambda: self.start_range("roof"))
                ui.Button("Whole leg", clicked_fn=self.whole_view)
                ui.Button("Knee close-up", clicked_fn=self.close_view)
            with ui.HStack(height=29):
                ui.Button("Rebuild stiffness", clicked_fn=lambda: self.launch(self.rebuild()))
                ui.Button("Save calculation", clicked_fn=self.save)
                ui.Button("Return to reference FEM", clicked_fn=lambda: asyncio.ensure_future(self.restore()))
            with ui.HStack(height=27):
                ui.Button("Replay recorded impact", clicked_fn=lambda: self.launch(self.replay_recorded()))
                ui.Button("Show peak bending", clicked_fn=self.show_peak)
            self.telemetry = ui.Label("", height=112, word_wrap=True)
            self.feedback = ui.Label("Apply and Cable demo solve forces; Drop solves impact. Full travel is not guaranteed by a force input.", height=64, word_wrap=True)
            with ui.ScrollingFrame(), ui.VStack(spacing=6, height=0):
                ui.Label("INPUTS: stiffness changes require Rebuild", height=25)
                self.inputs = {}
                for label, value in (("Panel / crease bending ratio", 100), ("Crease twist coupling", .1),
                                     ("Panel bending scale", 1), ("Membrane stiffness scale", 1),
                                     ("Membrane strain guard (%)", 3), ("Bend target (deg)", 30),
                                     ("Twist target (deg)", 60), ("Compression target (%)", 70),
                                     ("Cable tension (N)", .25), ("Roof test force (N)", 1),
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
                         "Discrete intersection check is NOT finite-thickness self-contact/CCD.\n"
                         "No fracture, delamination, cable-guide contact or strength rating.\n"
                         "Drop: assumed nodal/rigid inertia; backward Euler adds numerical damping.",
                         height=115, word_wrap=True)
                ui.Label("DISPLACEMENT STUDIES ONLY - not cable-force predictions", height=25)
                with ui.HStack(height=29):
                    for title, mode in (("Bend range", "bend"), ("Twist range", "twist"), ("Compression range", "compression")):
                        ui.Button(title, clicked_fn=lambda m=mode: self.start_range(m))

    def make_scene(self):
        s, root, model = self.stage, self.owned_path, self.shell
        self.surface = mesh(s, root+"/FlexiblePLA_PET", model.points, model.mesh["triangles"])
        self.surface.CreateDisplayColorAttr()
        UsdGeom.Primvar(self.surface.GetDisplayColorAttr()).SetInterpolation(UsdGeom.Tokens.uniform)
        white = material(s, "NonlinearCreaseWhite", (.9, .94, 1))
        gold = material(s, "NonlinearFrameGold", (.95, .55, .08))
        red = material(s, "NonlinearCableRed", (.9, .012, .025))
        black = material(s, "NonlinearCableBlack", (.012, .012, .015))
        link = material(s, "NonlinearLink", (.06, .18, .22))
        steel = material(s, "NonlinearYoke", (.65, .73, .8))
        self.line_edges = np.asarray([(a, b) for chain in model.mesh["chains"] for a, b in zip(chain[:-1], chain[1:])])
        self.frame_edges = np.asarray([(a, b) for i in model.mesh["frame_line_ids"] for a, b in zip(model.mesh["chains"][i][:-1], model.mesh["chains"][i][1:])])
        self.lines = curves(s, root+"/Original76SubdividedCreases", len(self.line_edges), .00012, white)
        self.frames_curve = curves(s, root+"/OnlyRigidPerimeters", len(self.frame_edges), .0007, gold)
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
            for side in (-1, 1):
                cube(str(body.GetPath())+f"/PerimeterSupport_{side+1}", (side*model.width/2, 0, sign*.004),
                     (.002, .003, .008), steel)
            cube(str(body.GetPath())+"/RaisedYoke", (0, 0, sign*.008), (model.width+.002, .004, .003), steel)
            if frame == 0:
                cube(str(body.GetPath())+"/Thigh", (0, 0, .046), (.009, .012, .073), link)
            else:
                cube(str(body.GetPath())+"/Shank", (0, 0, -.042), (.009, .012, .067), link)
                cube(str(body.GetPath())+"/Foot", (.012, 0, -.083), (.05, .026, .009), link)

    def set_time(self, elapsed):
        self.elapsed = elapsed  # Base initialization only; our jobs own the release.

    def whole_view(self):
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[.34, -.51, .3], target=[0, 0, .125])

    def close_view(self):
        center = self.offset+self.shell.height/2 if self.shell else .132
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[.08, -.13, center+.045], target=[0, 0, center])

    def render(self):
        if self.owned is None:
            return
        report = self.shell.diagnostics(self.state)
        points = report["points"] + [0, 0, self.offset]
        set_points(self.surface, points)
        values = np.clip(report["principal_membrane_strain"]/self.shell.config.panel_strain_limit, 0, 1)
        colors = np.stack([.12+.88*values, .45+.25*(1-np.abs(2*values-1)), .95*(1-values)], axis=1)
        self.surface.GetDisplayColorAttr().Set(Vt.Vec3fArray.FromNumpy(colors.astype(np.float32)))
        set_points(self.lines, points[self.line_edges].reshape(-1, 3))
        set_points(self.frames_curve, points[self.frame_edges].reshape(-1, 3))
        state_tensor = self.shell._tensor(self.state)
        top = self.shell.frame_point(state_tensor, 0, self.shell.cable_top).numpy()+[0, 0, self.offset]
        bottom = self.shell.frame_point(state_tensor, 1, self.shell.cable_bottom).numpy()+[0, 0, self.offset]
        cable = np.stack([top, bottom], axis=1)
        set_points(self.axial, cable[:4].reshape(-1, 3))
        set_points(self.red, cable[4:].reshape(-1, 3))
        set_points(self.loaded_cables, cable.reshape(-1, 3))
        self.loaded_cables.GetWidthsAttr().Set((.00032*(self.tensions > 1e-10)).tolist())
        if self.tensions.max() > 1e-10:
            self.loaded_cables.MakeVisible()
        else:
            self.loaded_cables.MakeInvisible()
        if self.static_trace and self.drop is None:
            applied = np.asarray(self.static_trace[-1]["nodal_loads_n"]).sum(axis=0)
            center = points[self.shell.mesh["bottom"]].mean(axis=0)
            set_points(self.applied_force, np.stack([center, center+.002*applied]))
            self.applied_force.MakeVisible() if np.linalg.norm(applied) > 1e-10 else self.applied_force.MakeInvisible()
        else:
            self.applied_force.MakeInvisible()
        corners = top[:4].copy()
        set_points(self.top_cross, corners[[0, 2, 1, 3]])
        roof_warp = []
        for frame, op in enumerate(self.body_ops):
            q = self.state[self.shell.frame_start+6*frame:self.shell.frame_start+6*(frame+1)]
            rotation = Rotation.from_rotvec(q[3:]).as_matrix()
            center = self.shell.frame_centers[frame].numpy()+self.shell.width*q[:3]+[0, 0, self.offset]
            matrix = np.eye(4)
            matrix[:3, :3], matrix[3, :3] = rotation.T, center
            op.Set(Gf.Matrix4d(*matrix.ravel().tolist()))
            mask = np.isclose(self.shell.points[:, 2], self.shell.frame_centers[frame, 2].item(), atol=1e-12)
            roof_warp.append(float(np.abs((points[mask]-center) @ rotation[:, 2]).max()))
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
        self.telemetry.text = (f"{self.mode} | physical geometry 1x\n"
            f"Bend X/Y {angles[0]:.3f} / {angles[1]:.3f} deg | twist {angles[2]:.3f} deg\n"
            f"Compression {report['compression_fraction']*100:.3f}% | membrane strain {report['max_membrane_strain']*100:.3f}%\n"
            f"Upper/lower roof warp {roof_warp[0]*1e6:.2f} / {roof_warp[1]*1e6:.2f} um\n"
            f"Ground force {force:.3f} N | contact time +{clock:.3f} ms | max cable {self.tensions.max():.3f} N")
        self.legend.text = (f"{self.mode.split(';')[0]} | RIGID GOLD FRAMES | 1x\n"
                            f"Bend Y {angles[1]:.3f} deg | twist {angles[2]:.3f} deg | compression {report['compression_fraction']*100:.2f}%\n"
                            "Uncalibrated shell | strain, NOT failure | Ground lines: 2 mm/N")
        self.frames += 1

    def neutral_shell(self):
        if self.work and not self.work.done():
            self.work.cancel()
        self.work = None
        self.running = False
        self.state = np.zeros(self.shell.ndof)
        self.drop = None
        self.display_index = None
        self.tensions = np.zeros(12)
        self.offset = self.lab.scene.top_height-self.shell.height
        self.mode = "NEUTRAL"
        self.snapshots = []
        self.static_trace = []
        self.last_rejection = None
        self.render()
        self.feedback.text = "Neutral. Original crease geometry restored; roof interiors remain free."

    def launch(self, coroutine):
        if self.work and not self.work.done():
            coroutine.close()
            self.feedback.text = "A calculation is active. Use Neutral / cancel before starting another."
            return
        self.running = True
        self.work = asyncio.ensure_future(self.guard_job(coroutine))

    async def guard_job(self, coroutine):
        try:
            await coroutine
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            self.feedback.text = "NOT APPLIED: "+str(exc)
        finally:
            self.running = False

    async def wait_running(self):
        while not self.running:
            if self.closed:
                raise asyncio.CancelledError()
            await omni.kit.app.get_app().next_update_async()

    async def rebuild(self):
        get = lambda name: self.inputs[name].as_float
        config = ShellConfig(panel_to_crease_ratio=get("Panel / crease bending ratio"),
                             crease_twist_ratio=get("Crease twist coupling"), panel_bending_scale=get("Panel bending scale"),
                             membrane_scale=get("Membrane stiffness scale"), panel_strain_limit=get("Membrane strain guard (%)")/100)
        config.validate()
        self.feedback.text = "Assembling the new nonlinear shell; displayed result held until ready."
        material = dataclasses.replace(self.shell.material, hinge_gap_m=get("PET exposed gap (mm)")/1000)
        material.validate()
        candidate = await asyncio.to_thread(NonlinearShell, self.lab.model.source, material, config)
        self.shell = candidate
        self.state = np.zeros(candidate.ndof)
        self.drop = None
        self.display_index = None
        self.snapshots = []
        self.tensions = np.zeros(12)
        self.last_rejection = None
        self.mode = "NEUTRAL - NEW STIFFNESS"
        self.static_trace = []
        self.offset = self.lab.scene.top_height-candidate.height
        self.render()
        self.feedback.text = "Stiffness rebuilt. New controls apply to subsequent range/cable/drop calculations."

    def start_drop(self):
        self.timeline_action = "drop"
        self.launch(self.drop_job())

    async def drop_job(self):
        get = lambda name: self.inputs[name].as_float
        config = ShellImpactConfig(upper_mass_kg=get("Upper-side mass (g)")/1000,
            lower_mass_kg=get("Lower-side mass (g)")/1000, contact_stiffness_n_m=get("Contact stiffness (N/m)"),
            contact_damping_ns_m=get("Contact damping (N s/m)"), step_s=get("Drop timestep (us)")*1e-6,
            duration_s=get("Drop duration (ms)")*.001)
        candidate = ShellImpact(self.shell, config)
        self.drop, self.state = candidate, candidate.state.copy()
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
            await asyncio.to_thread(candidate.step)
            await self.wait_running()
            self.drop = candidate
            self.state = candidate.state.copy()
            self.snapshots.append(self.state.copy())
            self.render()
            await omni.kit.app.get_app().next_update_async()
        self.last_rejection = self.drop.last_candidate if "GUARD" in str(self.drop.reason) or "LIMIT" in str(self.drop.reason) else None
        self.feedback.text = self.drop.reason+"\nThis is an uncalibrated mechanical response, not a survival verdict."

    def start_range(self, mode):
        self.timeline_action = mode
        self.launch(self.range_job(mode))

    async def range_job(self, mode):
        jobs = []
        if mode in ("bend", "twist", "compression"):
            label = {"bend": "Bend target (deg)", "twist": "Twist target (deg)", "compression": "Compression target (%)"}[mode]
            target = self.inputs[label].as_float
            if not np.isfinite(target) or not 0 < target <= (90 if mode == "compression" else 150):
                raise ValueError("Use a positive target up to 90% compression or 150 degrees")
            steps = max(1, math.ceil(target/2))
            for value in np.linspace(0, target, steps+1)[1:]:
                coordinate = {"bend": 4, "twist": 5, "compression": 2}[mode]
                si_value = self.shell.height*value/100 if mode == "compression" else np.deg2rad(value)
                jobs.append(({"lower_constraints": {coordinate: si_value}}, f"PRESCRIBED {mode.upper()} {value:.1f}; other coordinates free"))
        elif mode in ("cables", "manual"):
            amplitude = self.inputs["Cable tension (N)"].as_float
            if not np.isfinite(amplitude) or not 0 <= amplitude <= 10:
                raise ValueError("Experimental cable tension must be 0-10 N")
            families = ("Compression", "Bend X+", "Bend X-", "Bend Y+", "Bend Y-", "Twist CW", "Twist CCW")
            force = np.zeros(3)
            if mode == "manual":
                index = self.cable_pattern.model.get_item_value_model().as_int
                families = (families[index],)
                force = np.array([self.inputs[f"Lower frame force {axis} (N)"].as_float for axis in "XYZ"])
                if not np.isfinite(force).all() or np.abs(force).max() > 100:
                    raise ValueError("Use finite experimental frame-force components within +/-100 N")
            nodes = self.shell.mesh["bottom"]
            for family in families:
                fractions = np.linspace(0, 1, 9)[1:].tolist() if mode == "manual" else (.25, .5, .75, 1, .5, 0)
                for fraction in fractions:
                    loads = np.zeros_like(self.shell.points)
                    loads[nodes] = fraction*force/len(nodes)
                    jobs.append(({"tensions": tension_pattern(family, amplitude*fraction), "nodal_loads": loads},
                                 "FORCE-DRIVEN: "+family))
        else:
            amplitude = self.inputs["Roof test force (N)"].as_float
            if not np.isfinite(amplitude) or not 0 <= amplitude <= 10:
                raise ValueError("Experimental roof force must be 0-10 N")
            nodes = self.shell.free_nodes[np.isclose(self.shell.points[self.shell.free_nodes, 2], self.shell.height)]
            center = nodes[np.argmin(np.linalg.norm(self.shell.points[nodes, :2], axis=1))]
            load = np.zeros_like(self.shell.points)
            load[center, 2] = -amplitude
            jobs.append(({"nodal_loads": load}, f"ROOF CENTER FORCE {amplitude:g} N; PERIMETER FIXED"))
        # Invalid inputs must leave the last accepted physical state intact.
        self.state = np.zeros(self.shell.ndof)
        self.drop, self.last_rejection = None, None
        self.display_index = None
        self.offset = self.lab.scene.top_height-self.shell.height
        self.snapshots, self.static_trace = [], []
        self.tensions = np.zeros(12)
        self.close_view()
        for parameters, label in jobs:
            await self.wait_running()
            self.mode = label
            self.feedback.text = "Solving "+label+"; last accepted shape remains visible."
            state, report = await asyncio.to_thread(self.shell.solve, initial=self.state, **parameters)
            await self.wait_running()
            if not report["accepted"]:
                self.last_rejection = report
                self.feedback.text = (f"STOPPED: candidate NOT applied. Residual {report['gradient_max_j_per_scaled_coordinate']:.3g}; "
                    f"strain {report['max_membrane_strain']*100:.2f}%; intersections {len(report['surface_intersection_pairs'])}.\n"
                    "Requested full range has NOT been established. Last accepted state held.")
                return
            self.state = state
            self.tensions = np.asarray(report["tensions_n"])
            self.snapshots.append(state.copy())
            self.static_trace.append({"mode": label, "tensions_n": self.tensions.tolist(),
                                      "nodal_loads_n": np.asarray(parameters.get("nodal_loads", np.zeros_like(self.shell.points))).tolist(),
                                      "relative_rotation_rad": report["relative_rotation_rad"].tolist(),
                                      "compression_fraction": report["compression_fraction"],
                                      "max_membrane_strain": report["max_membrane_strain"],
                                      "residual": report["gradient_max_j_per_scaled_coordinate"]})
            self.render()
            reaction = report["required_frame_reactions_world_n_nm"][1]
            if mode in ("manual", "cables", "roof"):
                self.feedback.text = (f"Converged. Cable peak {max(self.tensions):.3g} N/strand. "
                    f"Residual {report['gradient_max_j_per_scaled_coordinate']:.2g} J/scaled coordinate.\n"
                    "Shape and strain are solved at these loads; no angle is imposed.")
            else:
                self.feedback.text = (f"Prescribed study. Required moment X/Y/Z: "
                    f"{reaction[3]:.4g} / {reaction[4]:.4g} / {reaction[5]:.4g} N m.")
            await omni.kit.app.get_app().next_update_async()
        self.feedback.text = "Requested calculation completed within current numerical guards; physical calibration still required."

    def show_peak(self):
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
        if self.drop is None or not self.snapshots:
            raise ValueError("Run a drop first; no recorded impact is available")
        self.mode = "RECORDED IMPACT REPLAY"
        self.offset = .0825
        self.close_view()
        for index, state in enumerate(self.snapshots):
            await self.wait_running()
            self.state = state.copy()
            self.display_index = index+1
            self.render()
            until = time.perf_counter()+.065
            while time.perf_counter() < until:
                await omni.kit.app.get_app().next_update_async()
        self.feedback.text = "Recorded solved frames replayed at about 15 frames/s; physical time is shown above."

    async def loop(self):
        try:
            while not self.closed:
                await omni.kit.app.get_app().next_update_async()
                if self.stage != omni.usd.get_context().get_stage():
                    self.running = False
                    if self.work:
                        self.work.cancel()
                    self.feedback.text = "Scene changed. Calculation halted; return to the reference workshop."
                    continue
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
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        folder = self.lab.project/"exact_joint/results"/(stamp+"_nonlinear_shell")
        folder.mkdir(parents=True, exist_ok=False)
        convert = lambda value: value.tolist() if isinstance(value, np.ndarray) else str(value)
        report = {"source_sha256": self.shell.source["sha256"], "material": dataclasses.asdict(self.shell.material),
                  "shell_config": dataclasses.asdict(self.shell.config), "mode": self.mode,
                  "inspected_trace_index": self.display_index,
                  "tensions_n": self.tensions.tolist(), "static_trace": self.static_trace,
                  "state": self.state, "snapshots": self.snapshots, "current": self.shell.diagnostics(self.state),
                  "last_rejected_candidate": self.last_rejection, "survives": None}
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
        if self.work:
            self.work.cancel()
            try:
                await self.work
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
