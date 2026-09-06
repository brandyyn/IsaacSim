"""Load fresh exact-joint code into the running Kit instance and open at neutral."""

import hashlib
import importlib
import os
import sys
import types
from pathlib import Path

project=Path(os.environ["PANEL_CREASE_PROJECT_ROOT"])
if str(project) not in sys.path:
    sys.path.insert(0,str(project))
importlib.invalidate_caches()
if "leg_redesign.app" in sys.modules:
    old=sys.modules["leg_redesign.app"]
    if old.ACTIVE_LAB:
        await old.ACTIVE_LAB.close()
        old.ACTIVE_LAB=None
if "exact_joint.app" in sys.modules:
    old=sys.modules["exact_joint.app"]
    if old.ACTIVE:
        await old.ACTIVE.close()
        old.ACTIVE=None
import exact_joint
for name in ("mesh_utils","geometry","elements","mechanics","scene","live_display","app"):
    qualified="exact_joint."+name
    module=sys.modules.get(qualified) or types.ModuleType(qualified)
    sys.modules[qualified]=module
    setattr(exact_joint,name,module)
    path=project/"exact_joint"/(name+".py")
    source=path.read_bytes()
    module.__file__=str(path)
    module.__package__="exact_joint"
    exec(compile(source,str(path),"exec"),module.__dict__)
    module.LOADED_SOURCE_SHA256=hashlib.sha256(source).hexdigest()
await exact_joint.app.open_workshop(project)
print("Opened one exact JSON knee joint with cable-load FEM and rigid leg links")
