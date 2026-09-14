"""Reload only the owned nonlinear workshop, preserving the original stage."""

import hashlib
import importlib
import sys
import types

from exact_joint import app
import exact_joint

lab = app.ACTIVE
if lab is None:
    raise RuntimeError("Open the exact-joint workshop first")
if lab.drop_preview is not None:
    await lab.drop_preview.restore()
importlib.invalidate_caches()
loaded = {}
for name in ("shell_ipc", "shell_sparse", "nonlinear_shell", "shell_impact", "shell_ui_jobs", "shell_view"):
    qualified = "exact_joint."+name
    module = sys.modules.get(qualified) or types.ModuleType(qualified)
    sys.modules[qualified] = module
    setattr(exact_joint, name, module)
    path = lab.project/"exact_joint"/(name+".py")
    source = path.read_bytes()
    module.__file__ = str(path)
    module.__package__ = "exact_joint"
    exec(compile(source, str(path), "exec"), module.__dict__)
    module.LOADED_SOURCE_SHA256 = hashlib.sha256(source).hexdigest()
    loaded[name] = module.LOADED_SOURCE_SHA256
await lab.open_nonlinear_shell()
print("Loaded updated shell:", loaded)
print("Active:", lab.drop_preview.shell.config)
