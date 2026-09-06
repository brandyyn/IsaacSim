"""Interactive exact-joint cable-tension FEM workshop inside Isaac Sim."""

import asyncio
import dataclasses
import json
import math
import time
from datetime import datetime,timezone
from pathlib import Path

import carb.settings
import numpy as np
import omni.kit.app
import omni.ui as ui
import omni.usd
from isaacsim.core.experimental.utils import app as app_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.rendering_manager import ViewportManager

from exact_joint.geometry import JointConfig,load_source
from exact_joint.mechanics import CableFem,tension_pattern
from exact_joint.scene import ExactLegScene

ACTIVE=None
MODES=("Compression","Bend X+","Bend X-","Bend Y+","Bend Y-","Twist CW","Twist CCW")


class Workshop:
    """Solve one cable-loaded original knee between unloaded rigid leg links."""

    def __init__(self,project,config):
        self.project=Path(project)
        self.config=config
        self.model=None
        self.scene=None
        self.results={}
        self.commands={"Knee":{"mode":"Compression","tension_n":0.0}}
        self.accepted_commands={}
        self.status={}
        self.window=None
        self.task=None
        self.demo=False
        self.paused=False
        self.dirty=True
        self.phase=0.0
        self.show_stress=False
        self.stress_scale_pa=2e7
        self.solve_ms=0.0
        self.error=None
        self.frames=0

    async def initialize(self):
        app_utils.stop(commit=False)
        await stage_utils.create_new_stage_async()
        self.scene=ExactLegScene(omni.usd.get_context().get_stage(),self.model)
        settings=carb.settings.get_settings()
        settings.set("/rtx/rendermode","RayTracedLighting")
        settings.set("/rtx/post/tonemap/op",4)
        self.build_ui()
        # Register the new window before docking it into the existing layout.
        await omni.kit.app.get_app().next_update_async()
        stage_window=next((window for window in ui.Workspace.get_windows() if window.title=="Stage"),None)
        if stage_window is not None:
            self.window.dock_in(stage_window,ui.DockPosition.SAME)
        self.window.focus()
        self.calculate()
        self.camera("Knee")
        self.task=asyncio.ensure_future(self.loop())

    def camera(self,joint=None):
        if joint is None:
            ViewportManager.set_camera_view("/OmniverseKit_Persp",eye=[.25,-.34,.21],target=[0,0,.12])
        else:
            target=self.scene.last_centers.get(joint,np.array([0,0,.145]))
            ViewportManager.set_camera_view("/OmniverseKit_Persp",eye=(target+[.060,-.080,.048]).tolist(),target=target.tolist())

    def command(self,joint,mode,tension):
        tension_pattern(mode,tension)
        self.commands[joint]={"mode":mode,"tension_n":float(tension)}
        self.demo=False
        self.paused=False
        self.dirty=True
        self.feedback.text=f"Solving {mode}: {tension:.3f} N / active cable..."

    def apply_ui_command(self,joint,mode,tension):
        """Report invalid UI inputs without losing the last valid result."""
        try:
            self.command(joint,mode,tension)
        except ValueError as exc:
            self.reject_command("Cable input rejected: "+str(exc),tension=tension)

    def reject_command(self, reason: str, tension: float | None = None) -> None:
        """Keep the last accepted result and put the rejection beside Apply."""
        self.error=reason
        self.demo=False
        self.paused=True
        requested=self.commands.get("Knee",{})
        tension=requested.get("tension_n",0) if tension is None else tension
        text=(f"NOT APPLIED: {tension:.3f} N / cable. {reason}\nLast accepted result is still displayed.")
        self.feedback.text=text
        self.message.text=text

    def calculate(self):
        started=time.perf_counter()
        if any(not handle.GetPrim().IsValid() for handles in self.scene.modules.values()
               for handle in handles.values() if hasattr(handle,"GetPrim")):
            raise RuntimeError("Scene disconnected. Save scene edits, then Rebuild exact joint FEM.")
        next_results={}
        for name,command in self.commands.items():
            initial=self.results[name]["q"] if name in self.results else np.zeros(6)
            result=self.model.solve(tension_pattern(command["mode"],command["tension_n"]),initial=initial)
            if not result["stats"]["within_small_deformation_model"]:
                raise ValueError(name+": small-strain model limit exceeded. Try 0.10 N; large folds need nonlinear FEM.")
            next_results[name]=result
        self.scene.update(next_results,show_stress=self.show_stress,stress_scale_pa=self.stress_scale_pa)
        self.results=next_results
        self.accepted_commands={name:dict(command) for name,command in self.commands.items()}
        self.error=None
        self.solve_ms=(time.perf_counter()-started)*1000
        self.frames+=1
        self.dirty=False
        self.feedback.text=(f"APPLIED - update {self.frames}. Read the values below. "
                            "Motion is true scale (micrometres); use Stress / material to see the stress field.")
        for name,result in self.results.items():
            stats=result["stats"]
            command=self.commands[name]
            self.status[name].text=(f"LAST ACCEPTED: {command['mode']} | {command['tension_n']:.3f} N / active cable\n"
                f"Bend X/Y: {stats['bend_xy_deg'][0]:.4f} / {stats['bend_xy_deg'][1]:.4f} deg\n"
                f"Twist: {stats['twist_deg']:.4f} deg | compression: {stats['compression_m']*1e6:.2f} um\n"
                f"PET / PLA peak: {stats['pet_peak_pa']/1e6:.2f} / {stats['pla_peak_pa']/1e6:.2f} MPa\n"
                f"Max principal strain: {stats['max_principal_strain']*100:.3f}%")
        self.message.text=f"Live equilibrium update: {self.solve_ms:.1f} ms | {len(self.model.tets)} elements/module\nUnconverged mesh; no strength, fatigue or cable-motor rating."

    async def loop(self):
        previous=time.perf_counter()
        last_solve=previous
        try:
            while True:
                await omni.kit.app.get_app().next_update_async()
                now=time.perf_counter()
                dt=now-previous
                previous=now
                if self.paused:
                    continue
                if self.demo:
                    self.phase+=min(dt,.1)
                    sequence=("Compression","Bend X+","Bend X-","Bend Y+","Bend Y-","Twist CW","Twist CCW")
                    for i,name in enumerate(self.commands):
                        phase=self.phase+i*2
                        mode=sequence[int(phase/8)%len(sequence)]
                        tension=.125*(1-math.cos(2*math.pi*(phase%8)/8))
                        self.commands[name]={"mode":mode,"tension_n":tension}
                    self.dirty=True
                if self.dirty and now-last_solve>.2:
                    try:
                        self.calculate()
                        self.error=None
                    except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
                        self.reject_command(str(exc))
                    last_solve=time.perf_counter()
        except asyncio.CancelledError:
            pass

    def neutral(self):
        self.demo=False
        self.paused=False
        for name in self.commands:
            self.commands[name]["tension_n"]=0
        self.dirty=True
        self.feedback.text="Removing cable loads..."

    def start_demo(self):
        self.demo=True
        self.paused=False
        self.phase=0
        self.feedback.text="Cable demo: experimental 0-0.25 N / active cable; true-scale deformation."

    def toggle_stress(self):
        self.show_stress=not self.show_stress
        self.scene.update(self.results,show_stress=self.show_stress,stress_scale_pa=self.stress_scale_pa)

    def save(self):
        stamp=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        folder=self.project/"exact_joint/results"/stamp
        folder.mkdir(parents=True,exist_ok=False)
        payload={"config":dataclasses.asdict(self.config),"source_sha256":self.model.source["sha256"],
                 "commands":self.accepted_commands,"requested_commands":self.commands,
                 "results":{name:r["stats"] for name,r in self.results.items()},
                 "last_rejected_command_error":self.error,"update_ms":self.solve_ms,
                 "scope":"live custom quasistatic FEM inside Kit, not native PhysX dynamics or ML"}
        (folder/"result.json").write_text(json.dumps(payload,indent=2),encoding="utf-8")
        self.message.text="Saved "+folder.name
        return folder

    def build_ui(self):
        self.window=ui.Window("Exact knee - cable FEM",width=440,height=720)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=5,height=0):
                    ui.Label("YOUR EXACT KNEE JOINT",height=26,style={"font_size":20})
                    ui.Label("One knee only | 28 vertices / 50 panels / 76 edges",height=20)
                    ui.Label("PLA 0.4 mm / PET 80 um | rigid square roofs",height=20)
                    ui.Label("LIVE QUASISTATIC FEM - not dynamic/large-fold validation",height=25,word_wrap=True)
                    with ui.HStack(height=28):
                        ui.Button("Cable demo",clicked_fn=self.start_demo)
                        ui.Button("Pause",clicked_fn=lambda:setattr(self,"paused",True))
                        ui.Button("Neutral",clicked_fn=self.neutral)
                    with ui.HStack(height=26):
                        ui.Button("Whole leg",clicked_fn=lambda:self.camera())
                        ui.Button("Exact knee close-up",clicked_fn=lambda:self.camera("Knee"))
                        ui.Button("Stress / material",clicked_fn=self.toggle_stress)
                    for name in self.commands:
                        ui.Separator(height=7)
                        ui.Label(name.upper()+" - independent cable loads",height=22)
                        with ui.HStack(height=26):
                            combo=ui.ComboBox(0,*MODES)
                            field=ui.FloatField(width=80)
                            field.model.set_value(self.commands[name]["tension_n"] or .1)
                            ui.Label("N / cable",width=75)
                            ui.Button("Apply",width=55,clicked_fn=lambda key=name,box=combo,value=field.model:
                                      self.apply_ui_command(key,MODES[box.model.get_item_value_model().as_int],value.as_float))
                        self.feedback=ui.Label("Start with 0.10 N per active cable, then Apply.",height=78,word_wrap=True)
                        self.status[name]=ui.Label("Initializing...",height=100,word_wrap=True)
                    ui.Separator(height=7)
                    ui.Label("Material/mesh edits need Rebuild, not Apply",height=24)
                    fields={}
                    for label,value in (("PET exposed gap (mm)",self.config.hinge_gap_m*1000),
                                        ("PET E (GPa)",self.config.pet_modulus_pa/1e9),
                                        ("PLA E (GPa)",self.config.pla_modulus_pa/1e9),
                                        ("Refinement 1-4",self.config.refinement)):
                        with ui.HStack(height=23):
                            ui.Label(label,width=230)
                            field=ui.FloatField()
                            field.model.set_value(value)
                            fields[label]=field.model
                    def rebuild():
                        config=dataclasses.replace(self.config,hinge_gap_m=fields["PET exposed gap (mm)"].as_float/1000,
                            pet_modulus_pa=fields["PET E (GPa)"].as_float*1e9,pla_modulus_pa=fields["PLA E (GPa)"].as_float*1e9,
                            refinement=int(fields["Refinement 1-4"].as_float))
                        try:
                            config.validate()
                            self.save()
                            asyncio.ensure_future(open_workshop(self.project,config))
                        except ValueError as exc:
                            self.message.text=str(exc)
                    ui.Button("Rebuild exact joint FEM",height=27,clicked_fn=rebuild)
                    ui.Button("Save FEM results and inputs",height=27,clicked_fn=self.save)
                    self.message=ui.Label("",height=70,word_wrap=True)
                    ui.Label("Tensions are experimental, not a hardware recommendation.\n"
                             "No gravity/payload, inertia, contact, plasticity or fatigue.\n"
                             "Large folds require nonlinear crease calibration.",height=65,word_wrap=True)

    async def close(self):
        if self.task:
            self.task.cancel()
            await self.task
        if self.window:
            self.window.visible=False
            self.window.destroy()


async def open_workshop(project,config=None):
    global ACTIVE
    config=config or JointConfig()
    config.validate()
    candidate=Workshop(project,config)
    try:
        source=load_source(candidate.project/"exact_joint/source_joint.json",config.width_m)
        candidate.model=CableFem(source,config)
    except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
        if ACTIVE and ACTIVE.window:
            ACTIVE.message.text="Rebuild rejected; old stage retained: "+str(exc)
            return ACTIVE
        raise
    if ACTIVE:
        await ACTIVE.close()
    ACTIVE=candidate
    await ACTIVE.initialize()
    return ACTIVE
