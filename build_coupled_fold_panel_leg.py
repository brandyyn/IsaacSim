"""Build the geometry-coupled original panel knee.

The paper specifies one revolute knee. This stage therefore starts from the
stable one-actuator leg and adds the shared-vertex/facet metadata needed by
the original 50-panel shell solver. It deliberately does not add the old
independent X/Z/Y gimbal: that was a presentation device, not the joint in
the supplied reference hardware.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import runpy
from pathlib import Path

from pxr import Sdf, Usd


STABLE = runpy.run_path(
    str(Path(__file__).with_name("build_stable_panel_crease_leg.py")),
    run_name="stable_panel_crease_leg_builder_for_geometry_coupled_model",
)
_source_to_joint = STABLE["HELPERS"]["_source_to_joint"]


def _validate_profile(profile: dict) -> None:
    samples = profile.get("samples", [])
    if len(samples) < 2:
        raise RuntimeError("coupled fold profile must contain at least two samples")
    commands = [float(sample["foldDeg"]) for sample in samples]
    if any(first >= second for first, second in zip(commands, commands[1:])):
        raise RuntimeError("coupled fold profile foldDeg values must be strictly increasing")
    for sample in samples:
        for key in ("compressionFraction",):
            if key not in sample:
                raise RuntimeError(f"coupled fold profile sample is missing {key}")


def _distance(first, second) -> float:
    return math.sqrt(
        sum((float(first[index]) - float(second[index])) ** 2 for index in range(3))
    )


def _author_constraint_metadata(stage: Usd.Stage, topology: dict) -> None:
    """Embed the source crease graph for the facet-rigid visual controller."""

    vertices_source = {
        str(item["id"]): [float(value) for value in item["positionM"]]
        for item in topology["geometry"]["vertices"]
    }
    vertices_joint = {
        vertex_id: [float(value) for value in _source_to_joint(position)]
        for vertex_id, position in vertices_source.items()
    }
    edge_constraints = []
    for line in topology["lines"]:
        vertex_ids = [str(value) for value in line["vertexIds"]]
        if len(vertex_ids) != 2:
            continue
        start, end = vertex_ids
        edge_constraints.append(
            {
                "name": line["name"],
                "a": start,
                "b": end,
                "restLengthM": _distance(vertices_joint[start], vertices_joint[end]),
            }
        )
    if len(edge_constraints) != 76:
        raise RuntimeError(f"expected 76 source crease edges, found {len(edge_constraints)}")

    top_ids = [
        vertex_id
        for vertex_id, position in vertices_source.items()
        if position[1] > 1.0e-8
    ]
    bottom_ids = [
        vertex_id
        for vertex_id, position in vertices_source.items()
        if position[1] < -1.0e-8
    ]
    central_ids = [
        vertex_id
        for vertex_id, position in vertices_source.items()
        if abs(position[1]) <= 1.0e-8
    ]
    facets = []
    source_facet_count = 0
    for panel in topology.get("panels", []):
        vertex_ids = [str(value) for value in panel.get("vertexIds", [])]
        if len(vertex_ids) not in (3, 4):
            continue
        source_facet_count += 1
        name = str(panel.get("name", "facet"))
        if len(vertex_ids) == 3:
            facets.append({"name": name, "vertexIds": vertex_ids})
        else:
            facets.extend(
                [
                    {"name": f"{name}_tri0", "vertexIds": vertex_ids[:3]},
                    {
                        "name": f"{name}_tri1",
                        "vertexIds": [vertex_ids[0], vertex_ids[2], vertex_ids[3]],
                    },
                ]
            )
    if source_facet_count != 50 or len(facets) != 52:
        raise RuntimeError(
            f"expected 50 source facets / 52 solver triangles, found {source_facet_count} / {len(facets)}"
        )

    visual = stage.GetPrimAtPath("/World/PanelCreaseLeg/OriginalJointVisual")
    visual.SetCustomDataByKey(
        "constraintEdgePairs", json.dumps(edge_constraints, separators=(",", ":"))
    )
    visual.SetCustomDataByKey(
        "constraintTopVertexIds", json.dumps(top_ids, separators=(",", ":"))
    )
    visual.SetCustomDataByKey(
        "constraintBottomVertexIds", json.dumps(bottom_ids, separators=(",", ":"))
    )
    visual.SetCustomDataByKey(
        "constraintCentralVertexIds", json.dumps(central_ids, separators=(",", ":"))
    )
    visual.SetCustomDataByKey(
        "constraintFacetTriples", json.dumps(facets, separators=(",", ":"))
    )
    visual.SetCustomDataByKey(
        "constraintModel",
        "50 source panels, 28 shared vertices, 76 source-edge constraints, hard flat-interface anchors",
    )
    visual.SetCustomDataByKey("constraintController", "facet_crease_panel_controller.py")
    visual.SetCustomDataByKey("controller", "facet_crease_panel_controller.py")

    controller = stage.GetPrimAtPath("/World/PanelCreaseLeg/Controller")
    controller.CreateAttribute("motionMode", Sdf.ValueTypeNames.String).Set(
        "single revolute knee with geometry-constrained 50-panel original shell"
    )
    controller.GetAttribute("originalJointModel").Set(
        "48 source triangular panels plus 2 rigid interface quads, welded through 28 shared crease vertices"
    )
    controller.CreateAttribute("constraintSolverIterations", Sdf.ValueTypeNames.Int).Set(4)
    controller.CreateAttribute("constraintObjectivePower", Sdf.ValueTypeNames.Float).Set(4.0)
    # Preserve measured source-edge lengths first. The facet metric is
    # reported for calibration, but a large facet-fit weight makes the closed
    # shell visibly stretch when an invalid end pose is requested.
    controller.CreateAttribute("constraintFacetFitWeight", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("constraintLineSearchSteps", Sdf.ValueTypeNames.Int).Set(4)
    controller.CreateAttribute("constraintMaxStepM", Sdf.ValueTypeNames.Float).Set(0.0015)
    controller.CreateAttribute("constraintPositionRegularization", Sdf.ValueTypeNames.Float).Set(1.0e-8)
    controller.CreateAttribute("constraintSelfClearanceM", Sdf.ValueTypeNames.Float).Set(0.0012)
    controller.CreateAttribute("constraintSelfClearanceWeight", Sdf.ValueTypeNames.Float).Set(16.0)
    controller.CreateAttribute("constraintMidpointClearanceM", Sdf.ValueTypeNames.Float).Set(0.0010)
    controller.CreateAttribute("constraintMidpointClearanceWeight", Sdf.ValueTypeNames.Float).Set(4.0)
    controller.CreateAttribute("constraintSolveRateHz", Sdf.ValueTypeNames.Float).Set(2.0)
    controller.CreateAttribute("constraintTargetEpsilonM", Sdf.ValueTypeNames.Float).Set(1.0e-7)
    controller.CreateAttribute("constraintSolverPending", Sdf.ValueTypeNames.Bool).Set(False)
    controller.CreateAttribute("constraintSolverCount", Sdf.ValueTypeNames.Int).Set(0)
    controller.CreateAttribute("constraintLastSolveMs", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("constraintSourceFacetCount", Sdf.ValueTypeNames.Int).Set(source_facet_count)
    controller.CreateAttribute("constraintFacetCount", Sdf.ValueTypeNames.Int).Set(len(facets))
    controller.CreateAttribute("constraintSolverIterationsUsed", Sdf.ValueTypeNames.Int).Set(0)
    for name in (
        "constraintMaxEdgeStrain",
        "constraintMeanEdgeStrain",
        "constraintMaxEdgeErrorM",
        "constraintMeanEdgeErrorM",
        "constraintMaxFacetEdgeDistortion",
        "constraintMeanFacetEdgeDistortion",
        "constraintMaxFacetFitErrorM",
        "constraintMeanFacetFitErrorM",
        "constraintMaxAnchorErrorM",
        "constraintMaxSelfClearanceViolationM",
        "constraintMeanSelfClearanceViolationM",
    ):
        controller.CreateAttribute(name, Sdf.ValueTypeNames.Float).Set(0.0)


def build(source_usd: Path, topology_json: Path, profile_json: Path, output_usd: Path) -> None:
    STABLE["build"](source_usd, topology_json, output_usd)
    stage = Usd.Stage.Open(str(output_usd))
    if stage is None:
        raise RuntimeError(f"could not reopen generated stage {output_usd}")

    with profile_json.open("r", encoding="utf-8") as handle:
        profile = json.load(handle)
    _validate_profile(profile)

    with topology_json.open("r", encoding="utf-8") as handle:
        topology = json.load(handle)
    _author_constraint_metadata(stage, topology)

    visual = stage.GetPrimAtPath("/World/PanelCreaseLeg/OriginalJointVisual")
    visual.SetCustomDataByKey(
        "coupledFoldProfile",
        json.dumps(profile, separators=(",", ":")),
    )
    visual.SetCustomDataByKey("coupledFoldProfileSource", profile_json.name)

    controller = stage.GetPrimAtPath("/World/PanelCreaseLeg/Controller")
    # The stage opens at an exact neutral pose. Motion is selected by the
    # launcher so a no-motion inspection never inherits a previous sweep.
    controller.CreateAttribute("motionDriver", Sdf.ValueTypeNames.String).Set("geometry_fold")
    controller.CreateAttribute("motionMode", Sdf.ValueTypeNames.String).Set(
        "neutral-first single-knee fold with geometry-constrained original shell"
    )
    controller.GetAttribute("originalJointModel").Set(
        "50 source surfaces solved as shared rigid facets from one physical knee revolute"
    )
    # The deterministic facet-rigid controller is the showcase runtime. The
    # full numerical optimizer remains available as an explicit validation
    # mode; keeping it out of the per-frame showcase path is important when a
    # humanoid contains many copies of this joint.
    controller.CreateAttribute("visualControllerMode", Sdf.ValueTypeNames.String).Set("kinematic_facet")
    controller.GetAttribute("visualControllerScript").Set("baked_panel_crease_controller.py")
    controller.CreateAttribute("kinematicFacetPasses", Sdf.ValueTypeNames.Int).Set(2)
    controller.CreateAttribute("kinematicMetricPeriod", Sdf.ValueTypeNames.Int).Set(15)
    controller.CreateAttribute("visualUpdateEpsilonM", Sdf.ValueTypeNames.Float).Set(1.0e-8)
    controller.CreateAttribute("visualUpdateEpsilonQuat", Sdf.ValueTypeNames.Float).Set(1.0e-7)
    controller.CreateAttribute("visualUpdateCount", Sdf.ValueTypeNames.Int).Set(0)
    controller.CreateAttribute("deterministicMaxAnchorErrorM", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("deterministicMaxSelfClearanceViolationM", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("neutralSnapToleranceM", Sdf.ValueTypeNames.Float).Set(5.0e-5)
    controller.GetAttribute("jointNames").Set("Hip,Knee,Ankle")
    controller.GetAttribute("defaultTargetsDeg").Set("0,0,0")
    controller.CreateAttribute("motionDemoAmplitudesDeg", Sdf.ValueTypeNames.String).Set("0,15,0")
    controller.CreateAttribute("foldCommandDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("foldCommandMinDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    # The source shell is driven from the paper/video knee axis. Keep the
    # showcase within the current geometry-validated visual envelope; the
    # paper's 127-degree actuator stop remains metadata for the eventual
    # calibrated FEA model, not a license to force this shell through contact.
    controller.CreateAttribute("foldCommandMaxDeg", Sdf.ValueTypeNames.Float).Set(15.0)
    controller.CreateAttribute("foldDemoCenterDeg", Sdf.ValueTypeNames.Float).Set(7.5)
    controller.CreateAttribute("foldDemoAmplitudeDeg", Sdf.ValueTypeNames.Float).Set(7.5)
    controller.CreateAttribute("geometryFoldPhaseDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("geometryFoldEnvelopeDeg", Sdf.ValueTypeNames.Float).Set(15.0)
    controller.CreateAttribute("geometryFoldResponse", Sdf.ValueTypeNames.String).Set(
        "one-knee cosine command; shell vertices solved from shared source creases; no independent gimbal targets"
    )
    controller.CreateAttribute("motionDemoEnabled", Sdf.ValueTypeNames.Bool).Set(False)
    controller.CreateAttribute("motionDemoRateHz", Sdf.ValueTypeNames.Float).Set(0.025)
    controller.CreateAttribute("coupledLateralTargetDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("coupledTwistTargetDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("coupledKneeTargetDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("coupledCompressionFraction", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("twistSweepCenterDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("twistSweepAmplitudeDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("twistSweepRateHz", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("twistSweepFoldDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("twistSweepLateralDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("twistCommandDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("twistSweepLowerDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("twistSweepUpperDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    # Legacy full-motion attributes remain for comparison scripts; the
    # current launcher uses the single geometry-fold driver above.
    controller.CreateAttribute("fullMotionRateHz", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("fullMotionPhaseDeg", Sdf.ValueTypeNames.Float).Set(0.0)
    controller.CreateAttribute("fullMotionPattern", Sdf.ValueTypeNames.String).Set("showcase")
    controller.CreateAttribute("fullMotionCombinedScale", Sdf.ValueTypeNames.Float).Set(0.75)
    controller.CreateAttribute("fullMotionShowcaseScale", Sdf.ValueTypeNames.Float).Set(0.75)

    knee = stage.GetPrimAtPath("/World/PanelCreaseLeg/Physics/KneeActuator")
    if not knee.IsValid():
        raise RuntimeError("stable geometry-fold stage is missing Physics/KneeActuator")
    knee.GetAttribute("drive:angular:physics:targetPosition").Set(0.0)

    root = stage.GetPrimAtPath("/World/PanelCreaseLeg")
    root.SetCustomDataByKey("assetType", "geometry_coupled_original_panel_crease_knee_experiment")
    root.SetCustomDataByKey(
        "kneeMechanism",
        "one physical Y-axis knee revolute drives the two flat interfaces; the 50-panel shell follows its welded crease geometry",
    )
    root.SetCustomDataByKey("coupledProfileSource", profile_json.name)
    root.SetCustomDataByKey("geometryValidFoldLimitDeg", 15.0)
    root.SetCustomDataByKey(
        "geometryValidFoldLimitNote",
        "15 degrees is the current visual validation envelope for the uncalibrated source shell; the paper actuator stop is 127 degrees and requires FEA-calibrated shell response before extension",
    )
    root.SetCustomDataByKey(
        "twistSweepNote",
        "the current knee model has one physical revolute; lateral/twist gimbal targets are intentionally not authored because the supplied paper specifies a one-DOF knee and the videos do not identify independent torsional stops",
    )
    root.SetCustomDataByKey(
        "fullMotionNote",
        "geometry_fold mode drives one smooth knee command from exact neutral through the geometry-valid envelope; the showcase uses deterministic rigid-facet projection from the original shared topology, while the numerical solver remains validation-only",
    )
    root.SetCustomDataByKey(
        "runtimeVisualModel",
        "deterministic facet-rigid shared-vertex runtime with 50 source surfaces and 76 exact source crease segments; one physical knee drives the shell, one combined crease-prism mesh is updated from the same welded map, and the numerical optimizer remains validation-only; no fallback crease-cylinder layer is authored",
    )
    root.SetCustomDataByKey("baselineStage", "panel_crease_leg_v8.usd")

    stage.GetRootLayer().Export(str(output_usd))
    print(f"Wrote {output_usd}")
    print(f"Coupled fold profile: {profile['name']} with {len(profile['samples'])} samples")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=os.environ.get("ORIGINAL_JOINT_USD"))
    parser.add_argument("--topology", type=Path, default=os.environ.get("ORIGINAL_JOINT_TOPOLOGY_JSON"))
    parser.add_argument("--profile", type=Path, default=os.environ.get("PANEL_CREASE_COUPLED_PROFILE"))
    parser.add_argument("--output", type=Path, default=os.environ.get("PANEL_CREASE_COUPLED_OUTPUT"))
    args = parser.parse_args()
    if args.source is None or args.topology is None or args.profile is None or args.output is None:
        parser.error("provide --source/--topology/--profile/--output or set the project environment variables")
    build(args.source.resolve(), args.topology.resolve(), args.profile.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()
