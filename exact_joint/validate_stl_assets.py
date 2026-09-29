"""Dependency-light audit for the three supplied binary STL files.

This deliberately uses only the Python standard library so it can run before
Isaac Sim/NumPy is available.  The Kit-side ``stl_assets.py`` performs the
same checks and additionally builds the assembled soft/rigid triangle split.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
from pathlib import Path


ROOT = Path(__file__).with_name("assets") / "stl_joint_v1"
EXPECTED = {
    "Rigid.stl": {
        "sha256": "4024ffc36300685089293e2708a8b07dab6e63a89d850ec2f7166a84d2d0d5ef",
        "bytes": 22484,
        "triangles": 448,
    },
    "soft_and_rigid_sections.stl": {
        "sha256": "182956b662f61680372b6ac5d3e5478bece34de4f71ba57bb06b280d6b4d7f86",
        "bytes": 30484,
        "triangles": 608,
    },
    "Soft.stl": {
        "sha256": "60a900edd721c97526ca914448153059881a84287d553b4b2f91904e1dc0f6da",
        "bytes": 8084,
        "triangles": 160,
    },
}


def audit(path: Path, expected: dict) -> dict:
    data = path.read_bytes()
    if len(data) != expected["bytes"]:
        raise AssertionError(f"{path.name}: byte count changed")
    count = struct.unpack_from("<I", data, 80)[0]
    if count != expected["triangles"] or len(data) != 84 + 50 * count:
        raise AssertionError(f"{path.name}: invalid binary STL triangle count")
    digest = hashlib.sha256(data).hexdigest()
    if digest != expected["sha256"]:
        raise AssertionError(f"{path.name}: SHA-256 changed")
    low = [math.inf] * 3
    high = [-math.inf] * 3
    triangle_keys = []
    for index in range(count):
        offset = 84 + index * 50 + 12
        vertices = struct.unpack_from("<9f", data, offset)
        for vertex in range(3):
            point = vertices[vertex * 3:vertex * 3 + 3]
            for axis in range(3):
                low[axis] = min(low[axis], point[axis])
                high[axis] = max(high[axis], point[axis])
        a, b, c = (vertices[0:3], vertices[3:6], vertices[6:9])
        ab = tuple(b[i] - a[i] for i in range(3))
        ac = tuple(c[i] - a[i] for i in range(3))
        cross = (ab[1] * ac[2] - ab[2] * ac[1],
                 ab[2] * ac[0] - ab[0] * ac[2],
                 ab[0] * ac[1] - ab[1] * ac[0])
        if sum(value * value for value in cross) <= 1e-24:
            raise AssertionError(f"{path.name}: degenerate triangle {index}")
        triangle_keys.append(tuple(sorted(tuple(round(value, 5) for value in vertices[i:i + 3])
                                             for i in range(0, 9, 3))))
    return {"bytes": len(data), "triangles": count, "sha256": digest,
            "min": low, "max": high, "triangle_keys": triangle_keys}


def main() -> int:
    results = {name: audit(ROOT / name, expected) for name, expected in EXPECTED.items()}
    manifest = json.loads((ROOT / "manifest.json").read_text(encoding="utf-8"))
    for name, result in results.items():
        recorded = manifest["files"][name]
        assert recorded["sha256"] == result["sha256"]
        assert recorded["triangle_count"] == result["triangles"]
    assembled_keys = set(results["soft_and_rigid_sections.stl"]["triangle_keys"])
    soft_keys = results["Soft.stl"]["triangle_keys"]
    assert len(set(soft_keys)) == len(soft_keys)
    assert set(soft_keys).issubset(assembled_keys)
    assert results["soft_and_rigid_sections.stl"]["triangles"] == (
        results["Soft.stl"]["triangles"] + results["Rigid.stl"]["triangles"]
    )
    for result in results.values():
        result.pop("triangle_keys", None)
    print(json.dumps(results, indent=2))
    print("STL audit: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
