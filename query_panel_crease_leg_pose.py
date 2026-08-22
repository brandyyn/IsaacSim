import omni.usd
from isaacsim.core.experimental.utils import xform


stage = omni.usd.get_context().get_stage()
for name in [
    "Body",
    "Thigh",
    "OriginalJointShell/TopShell",
    "OriginalJointShell/BottomShell",
    "Shank",
    "Foot",
]:
    path = f"/World/PanelCreaseLeg/{name}"
    position, quaternion = xform.get_world_pose(path)
    if hasattr(position, "numpy"):
        position = position.numpy()
    if hasattr(quaternion, "numpy"):
        quaternion = quaternion.numpy()
    print(f"{name}_position={position.tolist()}")
    print(f"{name}_quat={quaternion.tolist()}")
