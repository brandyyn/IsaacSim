"""Render one original JSON knee joint between rigid upper and lower leg sections."""

import dataclasses
import json

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.utils import stage as stage_utils


def material(stage,name,color,opacity=1):
    mat=UsdShade.Material.Define(stage,"/World/Looks/"+name)
    shader=UsdShade.Shader.Define(stage,str(mat.GetPath())+"/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor",Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness",Sdf.ValueTypeNames.Float).Set(.45)
    shader.CreateInput("opacity",Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),"surface")
    return mat


def mesh(stage,path,points,faces,mat=None):
    result=UsdGeom.Mesh.Define(stage,path)
    result.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(points,dtype=np.float32)))
    result.CreateFaceVertexCountsAttr([len(face) for face in faces])
    result.CreateFaceVertexIndicesAttr([int(i) for face in faces for i in face])
    result.CreateSubdivisionSchemeAttr("none")
    result.CreateDoubleSidedAttr(True)
    if mat:
        UsdShade.MaterialBindingAPI.Apply(result.GetPrim()).Bind(mat)
    return result


def curves(stage,path,count,width,mat):
    result=UsdGeom.BasisCurves.Define(stage,path)
    result.CreateTypeAttr("linear")
    result.CreateWrapAttr("nonperiodic")
    result.CreateCurveVertexCountsAttr([2]*count)
    result.CreateWidthsAttr([width])
    result.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    UsdShade.MaterialBindingAPI.Apply(result.GetPrim()).Bind(mat)
    return result


def set_points(prim,points):
    prim.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.asarray(points,dtype=np.float32)))


class ExactLegScene:
    """Display solved FEM nodes, exact rigid plates and cable endpoints; no animation skin."""

    @classmethod
    def attach(cls, stage, model):
        """Bind a saved exact-knee scene without rebuilding its stage, lights or camera.

        The active workshop supplies material constants; USD alone is not a solver
        checkpoint. Reject incompatible source/topology before touching any geometry.
        """
        if stage is None:
            raise ValueError("Open the saved exact-knee scene first.")
        root="/World/ExactLeg/Knee"
        original=stage.GetPrimAtPath(root+"/Original48Panels")
        if not original or original.GetCustomDataByKey("source_sha256")!=model.source["sha256"]:
            raise ValueError("Opened scene is not this source JSON knee; stage left unchanged.")
        config=stage.GetPrimAtPath(root).GetCustomDataByKey("fem_config_json")
        if config and json.loads(config)!=dataclasses.asdict(model.config):
            raise ValueError("Saved FEM settings differ from the current workshop; restore its settings first.")
        if UsdGeom.GetStageMetersPerUnit(stage)!=1 or UsdGeom.GetStageUpAxis(stage)!="Z":
            raise ValueError("Saved knee must retain metre units and Z-up.")
        candidate=cls.__new__(cls)
        candidate.stage,candidate.model=stage,model
        candidate.top_height,candidate.link_length=.145,.075
        candidate.last_centers={}
        names={"panels":"Original48Panels","roofs":"RigidSourceRoofs","creases":"Original76Edges",
               "red":"RedCrossPlateCables","axial":"AxialCables","black":"BlackTopRouting",
               "stress":"FEMStress","pet":"PETFilm","pla":"PLA48Panels"}
        handles={"root":root}
        for key,name in names.items():
            prim=stage.GetPrimAtPath(root+"/"+name)
            schema=UsdGeom.Mesh if key in ("panels","roofs","stress","pet","pla") else UsdGeom.BasisCurves
            if not prim or not prim.IsA(schema):
                raise ValueError("Missing exact-knee render object: "+name)
            handles[key]=schema(prim)
        for key,faces in (("pet",model.pet_surface),("pla",model.pla_surface),
                          ("stress",np.concatenate([model.pet_surface,model.pla_surface]))):
            if (len(handles[key].GetPointsAttr().Get())!=len(model.points)
                    or not np.array_equal(handles[key].GetFaceVertexIndicesAttr().Get(),np.asarray(faces).ravel())):
                raise ValueError("Saved FEM mesh differs from the active workshop configuration.")
        candidate.modules={"Knee":handles}
        roof_points=np.asarray(handles["roofs"].GetPointsAttr().Get())
        for roof in model.source["roofs"]:
            ids=roof["vertices"]
            saved=roof_points[ids]
            neutral=model.source["points"][ids]
            if not np.allclose(np.linalg.norm(saved[:,None]-saved[None,:],axis=2),
                               np.linalg.norm(neutral[:,None]-neutral[None,:],axis=2),atol=1e-7,rtol=0):
                raise ValueError("Saved rigid-roof size does not match the active FEM.")
        candidate.links=[]
        for path in ("/World/ExactLeg/Link_0","/World/ExactLeg/Foot"):
            prim=stage.GetPrimAtPath(path)
            if not prim or not prim.GetAttribute("xformOp:transform"):
                raise ValueError("Missing rigid leg transform: "+path)
            op=UsdGeom.XformOp(prim.GetAttribute("xformOp:transform"))
            if path.endswith("Foot"):
                candidate.foot_op=op
            else:
                candidate.links.append(op)
        candidate.mats={"RoofBlue":UsdShade.Material.Get(stage,"/World/Looks/RoofBlue")}
        if not candidate.mats["RoofBlue"]:
            raise ValueError("Missing exact-knee roof material.")
        return candidate

    def __init__(self,stage,model):
        self.stage,self.model=stage,model
        self.modules={}
        self.top_height=.145
        self.link_length=.075
        stage.SetDefaultPrim(stage_utils.define_prim("/World","Xform"))
        UsdGeom.SetStageMetersPerUnit(stage,1)
        UsdGeom.SetStageUpAxis(stage,"Z")
        self.mats={name:material(stage,name,color,alpha) for name,color,alpha in (
            ("PanelBlue",(.18,.40,.72),1),("RoofBlue",(.22,.42,.78),.65),
            ("Film",(.65,.82,.92),1),("CreaseWhite",(.93,.96,1),1),
            ("CableRed",(.80,.008,.015),1),("CableBlack",(.008,.009,.013),1),
            ("Link",(.06,.18,.22),1),("Metal",(.65,.72,.75),1),("Ground",(.12,.15,.18),1))}
        for name in ("Knee",):
            root="/World/ExactLeg/"+name
            stage_utils.define_prim(root,"Xform")
            stage.GetPrimAtPath(root).SetCustomDataByKey("fem_config_json",json.dumps(dataclasses.asdict(model.config)))
            source=model.source
            panel_mesh=mesh(stage,root+"/Original48Panels",source["points"],
                            [p["vertices"] for p in source["sides"]],self.mats["PanelBlue"])
            panel_mesh.GetPrim().SetCustomDataByKey("source_sha256",source["sha256"])
            panel_mesh.GetPrim().SetCustomDataByKey("original_panel_names",Vt.StringArray([p["name"] for p in source["sides"]]))
            panel_mesh.MakeInvisible()
            pet_mesh=mesh(stage,root+"/PETFilm",model.points,model.pet_surface,self.mats["Film"])
            pla_mesh=mesh(stage,root+"/PLA48Panels",model.points,
                          model.pla_surface,self.mats["PanelBlue"])
            roofs=mesh(stage,root+"/RigidSourceRoofs",source["points"],
                       [p["vertices"] for p in source["roofs"]],self.mats["RoofBlue"])
            roofs.GetPrim().SetCustomDataByKey("mechanics","exact rigid roof constraints in custom FEM solver; not native PhysX bodies")
            creases=curves(stage,root+"/Original76Edges",76,.00012,self.mats["CreaseWhite"])
            red=curves(stage,root+"/RedCrossPlateCables",8,.00022,self.mats["CableRed"])
            axial=curves(stage,root+"/AxialCables",4,.00015,self.mats["CableBlack"])
            black=curves(stage,root+"/BlackTopRouting",2,.00025,self.mats["CableBlack"])
            surfaces=np.concatenate([model.pet_surface,model.pla_surface])
            stress_mesh=mesh(stage,root+"/FEMStress",model.points,surfaces)
            stress_mesh.CreateDisplayColorAttr()
            UsdGeom.Primvar(stress_mesh.GetDisplayColorAttr()).SetInterpolation(UsdGeom.Tokens.uniform)
            stress_mesh.MakeInvisible()
            self.modules[name]={"root":root,"panels":panel_mesh,"roofs":roofs,"creases":creases,
                                "red":red,"axial":axial,"black":black,"stress":stress_mesh,
                                "pet":pet_mesh,"pla":pla_mesh}
        self.links=[]
        for index in range(1):
            path=f"/World/ExactLeg/Link_{index}"
            prim=UsdGeom.Xform.Define(stage,path)
            op=prim.AddTransformOp()
            shape=Cube(path+"/Beam",sizes=1,translations=(0,0,-self.link_length/2),
                       scales=(.009,.012,self.link_length))
            UsdShade.MaterialBindingAPI.Apply(shape.prims[0]).Bind(self.mats["Link"])
            self.links.append(op)
        foot=UsdGeom.Xform.Define(stage,"/World/ExactLeg/Foot")
        self.foot_op=foot.AddTransformOp()
        foot_shape=Cube(str(foot.GetPath())+"/Sole",sizes=1,translations=(.012,0,-.008),scales=(.05,.026,.009))
        UsdShade.MaterialBindingAPI.Apply(foot_shape.prims[0]).Bind(self.mats["Link"])
        # A fork supports the fixed upper roof while leaving the central black-X
        # routing visible. Hip and ankle origami copies are intentionally absent.
        for side in (-1,1):
            fork=Cube(f"/World/ExactLeg/ThighFork_{side+1}",sizes=1,
                      translations=(0,side*.0105,self.top_height+.009),scales=(.004,.004,.018))
            UsdShade.MaterialBindingAPI.Apply(fork.prims[0]).Bind(self.mats["Metal"])
        brace=Cube("/World/ExactLeg/ThighBrace",sizes=1,translations=(0,0,self.top_height+.019),scales=(.009,.025,.004))
        UsdShade.MaterialBindingAPI.Apply(brace.prims[0]).Bind(self.mats["Metal"])
        thigh=Cube("/World/ExactLeg/Thigh",sizes=1,translations=(0,0,self.top_height+.047),scales=(.009,.012,.054))
        UsdShade.MaterialBindingAPI.Apply(thigh.prims[0]).Bind(self.mats["Link"])
        mount=Cube("/World/ExactLeg/Mount",sizes=1,translations=(0,0,.226),scales=(.028,.025,.016))
        UsdShade.MaterialBindingAPI.Apply(mount.prims[0]).Bind(self.mats["Metal"])
        floor=Cube("/World/Plinth",sizes=1,translations=(0,0,-.01),scales=(.25,.22,.01))
        UsdShade.MaterialBindingAPI.Apply(floor.prims[0]).Bind(self.mats["Ground"])
        UsdLux.DomeLight.Define(stage,"/World/Lighting/Dome").CreateIntensityAttr(900)
        key=UsdLux.DistantLight.Define(stage,"/World/Lighting/Key")
        key.CreateIntensityAttr(2400)
        UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(20,-30,-25))
        self.last_centers={}

    @staticmethod
    def transform(op,rotation,translation):
        value=np.eye(4)
        value[:3,:3]=rotation.T
        value[3,:3]=translation
        op.Set(Gf.Matrix4d(*value.ravel().tolist()))

    def update(self,results,*,show_stress=False,stress_scale_pa=2e7):
        model=self.model
        h=model.source["height_m"]
        parent_rotation=np.eye(3)
        top_center=np.array([0,0,self.top_height])
        source_indices=np.array([model.coarse_map[i] for i in range(28)])
        for index,(name,handles) in enumerate(self.modules.items()):
            result=results[name]
            local=result["points"][source_indices]
            world=(local-[0,0,h]) @ parent_rotation.T+top_center
            set_points(handles["panels"],world)
            set_points(handles["roofs"],world)
            all_world=(result["points"]-[0,0,h]) @ parent_rotation.T+top_center
            set_points(handles["pet"],all_world)
            set_points(handles["pla"],all_world)
            edges=np.array([line["vertices"] for line in model.source["lines"]])
            set_points(handles["creases"],world[edges].reshape(-1,3))
            top=(model.top_anchors-[0,0,h]) @ parent_rotation.T+top_center
            bottom=(result["bottom_anchors"]-[0,0,h]) @ parent_rotation.T+top_center
            cable=np.stack([top,bottom],axis=1)
            set_points(handles["axial"],cable[:4].reshape(-1,3))
            set_points(handles["red"],cable[4:].reshape(-1,3))
            corners=world[model.source["top"]]
            center=corners.mean(axis=0)
            order=np.argsort(np.arctan2((corners-center) @ parent_rotation[:,1],(corners-center) @ parent_rotation[:,0]))
            corners=corners[order]+parent_rotation[:,2]*.0003
            set_points(handles["black"],corners[[0,2,1,3]])
            if show_stress:
                handles["pet"].MakeInvisible()
                handles["pla"].MakeInvisible()
                handles["stress"].MakeVisible()
                set_points(handles["stress"],(result["points"]-[0,0,h]) @ parent_rotation.T+top_center)
                owners=model.surface_owners
                values=result["von_mises"][owners]
                v=np.clip(values/stress_scale_pa,0,1)
                colors=np.stack([v,.25+.65*(1-np.abs(2*v-1)),1-v],axis=1)
                handles["stress"].GetDisplayColorAttr().Set(Vt.Vec3fArray.FromNumpy(colors.astype(np.float32)))
            else:
                handles["pet"].MakeVisible()
                handles["pla"].MakeVisible()
                handles["stress"].MakeInvisible()
            bottom_center=(result["q"][:3]-[0,0,h]) @ parent_rotation.T+top_center
            self.last_centers[name]=(top_center+bottom_center)/2
            parent_rotation=parent_rotation @ result["rotation"]
            self.transform(self.links[index],parent_rotation,bottom_center)
            ankle_position=bottom_center+parent_rotation @ np.array([0,0,-self.link_length])
            self.transform(self.foot_op,parent_rotation,ankle_position)
