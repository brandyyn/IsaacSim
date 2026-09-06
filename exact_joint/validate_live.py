"""Run unit tests, mesh/gap sensitivity and live single-knee checks inside Kit."""

async def run_validation():
    import dataclasses
    import hashlib
    import io
    import json
    import sys
    import time
    import unittest
    from datetime import datetime,timezone
    import numpy as np
    import omni.kit.app
    import omni.usd
    from pxr import UsdGeom
    from exact_joint import app
    from exact_joint.geometry import JointConfig,load_source
    from exact_joint.mechanics import CableFem,tension_pattern

    lab=app.ACTIVE
    lab.demo=False
    lab.paused=True
    suite=unittest.defaultTestLoader.loadTestsFromName("exact_joint.test_exact_joint")
    stream=io.StringIO()
    tests=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    report={"unit_tests":{"count":tests.testsRun,"pass":tests.wasSuccessful(),"output":stream.getvalue()},
            "source_sha256":lab.model.source["sha256"],"config":dataclasses.asdict(lab.config),
            "source_hashes":{name:hashlib.sha256((lab.project/"exact_joint"/(name+".py")).read_bytes()).hexdigest()
                             for name in ("mesh_utils","geometry","elements","mechanics","scene","app")},
            "studies":[],"live_cases":[]}
    assert set(lab.scene.modules)=={"Knee"},"Only one origami knee may be active"
    assert not omni.usd.get_context().get_stage().GetPrimAtPath("/World/ExactLeg/Hip")
    assert not omni.usd.get_context().get_stage().GetPrimAtPath("/World/ExactLeg/Ankle")
    report["active_origami_joints"]=["Knee"]
    print("Unit tests",tests.testsRun,tests.wasSuccessful(),stream.getvalue())
    for gap,refinement in ((.0001,1),(.0002,1),(.0004,1),(.0002,2),(.0002,3)):
        config=dataclasses.replace(lab.config,hinge_gap_m=gap,refinement=refinement)
        model=CableFem(lab.model.source,config)
        for mode in ("Compression","Bend X+","Twist CW"):
            started=time.perf_counter()
            result=model.solve(tension_pattern(mode,.1),initial=np.zeros(6))
            report["studies"].append({"gap_m":gap,"refinement":refinement,"nodes":len(model.points),
                                      "elements":len(model.tets),"result":result["stats"],
                                      "solve_ms":(time.perf_counter()-started)*1000})
            print("Study",gap,refinement,mode,result["stats"]["compression_m"],result["stats"]["twist_deg"])
    for mode in ("Compression","Bend X+","Bend X-","Bend Y+","Bend Y-","Twist CW","Twist CCW"):
        for name in lab.commands:
            lab.command(name,mode,.25)
        lab.calculate()
        stage=omni.usd.get_context().get_stage()
        errors=[]
        for name,handles in lab.scene.modules.items():
            assert len(handles["creases"].GetCurveVertexCountsAttr().Get())==76
            coords=np.array(handles["roofs"].GetPointsAttr().Get())
            for roof in lab.model.source["roofs"]:
                ids=roof["vertices"]
                moved=coords[ids]
                rest=lab.model.source["points"][ids]
                errors.append(float(np.max(np.abs(np.linalg.norm(moved[:,None]-moved[None,:],axis=2)
                                                 -np.linalg.norm(rest[:,None]-rest[None,:],axis=2)))))
        report["live_cases"].append({"mode":mode,"roof_rigidity_error_m":max(errors),
                                      "stats":{name:r["stats"] for name,r in lab.results.items()},
                                      "solve_ms":lab.solve_ms})
        for _ in range(4):
            await omni.kit.app.get_app().next_update_async()
    lab.neutral()
    lab.calculate()
    lab.toggle_stress()
    lab.start_demo()
    start=time.perf_counter()
    frames=lab.frames
    while time.perf_counter()-start<10:
        await omni.kit.app.get_app().next_update_async()
    report["demo"]={"duration_s":time.perf_counter()-start,"fem_updates":lab.frames-frames,"error":lab.error}
    lab.demo=False
    lab.paused=True
    stamp=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    folder=lab.project/"exact_joint/results"/stamp
    folder.mkdir(parents=True,exist_ok=False)
    (folder/"validation.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print("VALIDATION_SAVED",folder)
    assert tests.wasSuccessful(),"Unit tests failed"
    assert len(report["live_cases"])==7,"Actuation sweep is incomplete"
    assert len(report["studies"])==15,"Mesh/gap sweep is incomplete"
    assert report["demo"]["error"] is None,"Live FEM demo rejected a load"
    assert report["demo"]["fem_updates"]>10,"Live FEM did not update"

await run_validation()
