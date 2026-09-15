"""Separate magnified vector plot and recorded-step replay; no physical writes."""

import time

import numpy as np
from pxr import UsdGeom
from scipy.spatial.transform import Rotation

from exact_joint.scene import curves, material, mesh, set_points
from exact_joint.shell_display_math import displacement_plot


class ShellDisplacementView:
    def __init__(self, view):
        self.view = view
        self.path = view.owned_path+"/DisplacementDiagnostic"
        self.root = UsdGeom.Xform.Define(view.stage, self.path)
        self.root.GetPrim().SetCustomDataByKey("scope", "DISPLAY ONLY displacement vector plot; no physics or strength prediction")
        self.root.GetPrim().SetCustomDataByKey("formula", "reference + gain*(upper_frame_aligned_solution-reference)")
        self.playing = False
        self.paused = False
        self.frames = []
        self.last_index = None
        neutral = material(view.stage, "ShellDiagnosticNeutral", (.5, .52, .55))
        cyan = material(view.stage, "ShellDiagnosticCyan", (.03, .75, .9))
        white = material(view.stage, "ShellDiagnosticEdges", (.95, .98, 1))
        self.surface = mesh(view.stage, self.path+"/MagnifiedDisplacement", view.shell.points,
                            view.shell.mesh["triangles"], cyan)
        self.reference = curves(view.stage, self.path+"/NeutralOutline", len(view.line_edges), .00010, neutral)
        self.edges = curves(view.stage, self.path+"/MagnifiedCreases", len(view.line_edges), .00014, white)
        self.max_displacement_m = 0.0
        self.update(view.state)

    @property
    def offset(self):
        # This diagnostic stays beside the original knee, not on the drop floor.
        return np.array([.060, 0, self.view.lab.scene.top_height-self.view.shell.height])

    def update(self, state, label="CURRENT SOLVED RESPONSE"):
        view, shell = self.view, self.view.shell
        self.root.MakeVisible() if view.show_response else self.root.MakeInvisible()
        if not view.show_response:
            return
        q = state[shell.frame_start:shell.frame_start+6]
        points = shell.positions(shell._tensor(state)).detach().numpy()
        plotted, displacement = displacement_plot(shell.points, points, shell.frame_centers[0].numpy(),
            shell.width*q[:3], Rotation.from_rotvec(q[3:]).as_matrix(), view.response_gain, self.offset)
        set_points(self.surface, plotted)
        set_points(self.reference, (shell.points+self.offset)[view.line_edges].reshape(-1, 3))
        set_points(self.edges, plotted[view.line_edges].reshape(-1, 3))
        self.max_displacement_m = float(np.linalg.norm(displacement, axis=1).max())
        self.root.GetPrim().SetCustomDataByKey("display_gain", float(view.response_gain))
        self.root.GetPrim().SetCustomDataByKey("replay_label", label)
        view.response_label.text = (f"RIGHT: displacement x{view.response_gain:g}, DISPLAY ONLY\n{label}\n"
            f"True displacement: {self.max_displacement_m*1e6:.2f} um | Grey = neutral")

    def replay(self):
        view = self.view
        if not view.ensure_current_scene():
            return
        if view.work and not view.work.done():
            view.feedback.text = "Wait for the current solve before replaying its recorded steps."
            return
        if not view.snapshots:
            view.feedback.text = "No accepted movement recorded yet. Choose a cable pattern first."
            return
        view.show_response = True
        view.response_visible.set_value(True)
        self.frames = [np.zeros(view.shell.ndof)]+[state.copy() for state in view.snapshots]
        self.started = time.perf_counter()
        self.last_index = None
        self.playing = True
        self.paused = False
        view.close_view()
        view.feedback.text = "Right-hand plot loops recorded accepted states at 2 steps/s. Left leg and FEM remain unchanged."

    def tick(self):
        if not self.playing or self.paused or not self.view.show_response:
            return
        # Hold both neutral and the final recorded state for one extra beat.
        count = len(self.frames)
        phase = int((time.perf_counter()-self.started)*2) % (count+2)
        index = max(0, min(count-1, phase-1))
        if index != self.last_index:
            self.last_index = index
            self.update(self.frames[index], f"REPLAY STEP {index}/{count-1}; NOT LIVE PHYSICAL TIME")

    def stop(self):
        self.playing = False
        self.paused = False
        if not self.view.ensure_current_scene():
            return
        self.update(self.view.state)

    def toggle_pause(self):
        if self.paused:
            self.started += time.perf_counter()-self.paused_at
            self.paused = False
            self.view.feedback.text = "Recorded diagnostic replay resumed; physical leg remains unchanged."
        else:
            self.paused_at = time.perf_counter()
            self.paused = True
            self.view.feedback.text = "PAUSED: recorded diagnostic held. Pause / resume continues; Hold shows the final solution."
