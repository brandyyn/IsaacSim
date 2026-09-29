"""Generate the checked-in provenance manifest with Isaac Sim's NumPy runtime."""

import json
from pathlib import Path

from exact_joint.stl_assets import ASSET_DIR, write_manifest


manifest_path = Path(__file__).with_name("assets") / "stl_joint_v1" / "manifest.json"
result = write_manifest(manifest_path, ASSET_DIR)
print(json.dumps(result, indent=2))

