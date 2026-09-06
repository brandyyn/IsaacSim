"""Load glyphs and a separate magnified FEM plot; never modify the physical result."""

import numpy as np
import omni.ui as ui
from omni.kit.viewport.utility import get_active_viewport_window
from pxr import UsdGeom, Vt

from exact_joint.scene import curves, material, mesh, set_points


class LiveFemDisplay:
    """Render solved displacement times a labelled gain, plus cable force-direction glyphs."""

    def __init__(self, scene) -> None:
        self.scene = scene
        self.model = scene.model
        self.stage = scene.stage
        self.root = "/World/ExactFemDisplay"
        prim = UsdGeom.Xform.Define(self.stage, self.root).GetPrim()
        prim.SetCustomDataByKey("purpose", "Diagnostic display only; not a second physical joint")
        prim.SetCustomDataByKey("deformation_formula", "x_display = x_reference + gain * u_FEM")
        self.offset = np.array([0.065, 0, scene.top_height - self.model.source["height_m"]])
        self.physical_offset = np.array([0, 0, self.offset[2]])
        self.indices = np.array([self.model.coarse_map[i] for i in range(28)])
        self.edges = np.array([line["vertices"] for line in self.model.source["lines"]])
        neutral = material(self.stage, "FemNeutral", (0.35, 0.38, 0.42))
        edge = material(self.stage, "FemPlotEdges", (0.94, 0.96, 1.0))
        load = material(self.stage, "ActiveCableLoad", (1.0, 0.48, 0.015))
        self.surface = mesh(self.stage, self.root + "/MagnifiedDisplacement",
                            self.model.points, np.concatenate([self.model.pet_surface, self.model.pla_surface]))
        self.surface.CreateDisplayColorAttr()
        UsdGeom.Primvar(self.surface.GetDisplayColorAttr()).SetInterpolation(UsdGeom.Tokens.uniform)
        self.roofs = mesh(self.stage, self.root + "/MagnifiedRoofOutlines", self.model.source["points"],
                          [roof["vertices"] for roof in self.model.source["roofs"]], scene.mats["RoofBlue"])
        self.crease = curves(self.stage, self.root + "/MagnifiedCreases", 76, 0.00012, edge)
        self.reference = curves(self.stage, self.root + "/NeutralReference", 76, 0.00008, neutral)
        set_points(self.reference, (self.model.source["points"] + self.offset)[self.edges].reshape(-1, 3))
        self.routes = curves(self.stage, self.root + "/MagnifiedCableRoutes", 12, 0.00015, neutral)
        self.loads = []
        for view in ("Actual", "Magnified"):
            self.loads.append([
                curves(self.stage, f"{self.root}/{view}CableLoad_{i}", 1, 0.0005, load) for i in range(12)
            ])
        self.arrows = curves(self.stage, self.root + "/ForceDirectionGlyphs", 0, 0.00035, load)
        self.arrow_spans = []
        self.gain = 500.0
        self.visible = True
        self.last_points = None
        self.last_tensions = np.zeros(12)
        self.hud_frame = get_active_viewport_window().get_frame("exact_joint_live_fem_legend")
        with self.hud_frame:
            with ui.VStack():
                ui.Spacer(height=64)
                with ui.HStack(height=84):
                    # Leave the viewport's vertical tool strip clear in recordings.
                    ui.Spacer(width=90)
                    with ui.ZStack():
                        ui.Rectangle(style={"background_color": 0xD9222428}, height=84)
                        with ui.VStack(spacing=3):
                            self.legend = ui.Label("", word_wrap=True, height=48,
                                                   style={"font_size": 18, "color": 0xFFFFFFFF})
                            self.telemetry = ui.Label("", height=28,
                                                      style={"font_size": 17, "color": 0xFF80DFFF})
                    ui.Spacer(width=25)
                ui.Spacer()

    def update(self, result: dict, gain: float, stress_max_pa: float, show_plot: bool = True) -> None:
        """Update annotations using accepted FEM data only (SI), leaving result arrays untouched."""
        self.gain = gain
        self.visible = show_plot
        deformed = self.model.points + gain * result["displacements"] + self.offset
        self.last_points = deformed
        self.last_tensions = result["tensions"].copy()
        set_points(self.surface, deformed)
        coarse = deformed[self.indices]
        set_points(self.roofs, coarse)
        set_points(self.crease, coarse[self.edges].reshape(-1, 3))
        values = result["von_mises"][self.model.surface_owners] / stress_max_pa
        v = np.clip(values, 0, 1)
        colors = np.stack([v, 0.25 + 0.65 * (1 - np.abs(2 * v - 1)), 1 - v], axis=1)
        self.surface.GetDisplayColorAttr().Set(Vt.Vec3fArray.FromNumpy(colors.astype(np.float32)))
        for prim in (self.surface, self.roofs, self.crease, self.reference, self.routes):
            prim.MakeVisible() if show_plot else prim.MakeInvisible()
        top = self.model.top_anchors
        bottom = result["bottom_anchors"]
        plot_bottom = self.model.bottom_anchors + gain * (bottom - self.model.bottom_anchors)
        spans = [np.stack([top + self.physical_offset, bottom + self.physical_offset], axis=1),
                 np.stack([top + self.offset, plot_bottom + self.offset], axis=1)]
        set_points(self.routes, spans[1].reshape(-1, 3))
        self.arrow_spans = []
        for view, cable_spans in enumerate(spans):
            for i, cable in enumerate(self.loads[view]):
                active = result["tensions"][i] > 1e-6 and (view == 0 or show_plot)
                if active:
                    cable.MakeVisible()
                    set_points(cable, cable_spans[i])
                    cable.GetWidthsAttr().Set([0.00025 + 0.0004 * min(result["tensions"][i] / 0.25, 1)])
                    self.arrow_spans.append(cable_spans[i])
                else:
                    cable.MakeInvisible()
        self.legend.text = (f"ACTUAL KNEE: 1x   |   RIGHT: FEM DISPLACEMENT {gain:.0f}x (DISPLAY ONLY)\n"
                            f"Stress: blue 0 - red {stress_max_pa/1e6:g}+ MPa; not a failure scale. Gold = loaded cable.")
        stats = result["stats"]
        self.telemetry.text = (f"TRUE RESPONSE: compression {stats['compression_m']*1e6:.2f} um  |  "
                               f"twist {stats['twist_deg']:.4f} deg  |  PET peak {stats['pet_peak_pa']/1e6:.2f} MPa")
        self.surface.GetPrim().SetCustomDataByKey("display_gain", float(gain))

    def animate(self, phase: float) -> None:
        """Moving chevrons show opposing cable force directions, NOT cable speed or dynamics."""
        points = []
        for top, bottom in self.arrow_spans:
            direction = top - bottom
            direction /= np.linalg.norm(direction)
            side = np.cross(direction, [0, 1, 0])
            if np.linalg.norm(side) < 0.1:
                side = np.cross(direction, [1, 0, 0])
            side /= np.linalg.norm(side)
            for sign in (1, -1):
                fraction = 0.12 + 0.28 * ((phase * 0.7) % 1)
                center = bottom + fraction * (top-bottom) if sign == 1 else top - fraction * (top-bottom)
                tip = center + sign * direction * 0.0014
                points.extend([center + side * 0.0008, tip, center - side * 0.0008])
        self.arrows.GetCurveVertexCountsAttr().Set([3] * (len(points) // 3))
        set_points(self.arrows, np.asarray(points).reshape(-1, 3))

    def close(self) -> None:
        self.hud_frame.clear()
