"""Validated STL assets for the photo-faithful knee reference.

The nonlinear FEM remains driven by ``source_joint.json``.  These helpers keep
the supplied CAD files byte-identical, validate their binary STL structure, and
split the assembled mesh into the exact soft and rigid triangle sets for a
reference overlay.  No STL-derived thickness or stiffness is silently fed into
the FEM solver.
"""

from __future__ import annotations

import hashlib
import json
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np


ASSET_DIR = Path(__file__).with_name("assets") / "stl_joint_v1"
SOURCE_UNIT_ASSUMPTION = "millimetre (binary STL has no embedded units; verify against measured part)"
STL_TO_METRE = 1.0e-3


@dataclass(frozen=True)
class StlMesh:
    name: str
    path: Path
    triangles: np.ndarray
    normals: np.ndarray
    sha256: str

    @property
    def triangle_count(self) -> int:
        return int(self.triangles.shape[0])

    @property
    def bounds(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        if self.triangles.size == 0:
            zero = (0.0, 0.0, 0.0)
            return zero, zero
        points = self.triangles.reshape(-1, 3)
        return tuple(np.min(points, axis=0).tolist()), tuple(np.max(points, axis=0).tolist())


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def read_binary_stl(path: str | Path) -> StlMesh:
    """Read and validate a binary STL without trusting its human header."""
    path = Path(path)
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path.name}: binary STL header is truncated")
    count = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + 50 * count
    if len(data) != expected:
        raise ValueError(f"{path.name}: expected {expected} bytes for {count} triangles, got {len(data)}")
    dtype = np.dtype([
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ])
    records = np.frombuffer(data, dtype=dtype, offset=84, count=count)
    triangles = np.asarray(records["vertices"], dtype=np.float64).copy()
    normals = np.asarray(records["normal"], dtype=np.float64).copy()
    if not np.isfinite(triangles).all() or not np.isfinite(normals).all():
        raise ValueError(f"{path.name}: non-finite vertex or normal")
    edges = triangles[:, [1, 2, 0], :] - triangles[:, [0, 0, 0], :]
    if np.any(np.linalg.norm(np.cross(edges[:, 0], edges[:, 1]), axis=1) <= 1e-12):
        raise ValueError(f"{path.name}: degenerate triangle")
    return StlMesh(path.stem, path, triangles, normals, _sha256(data))


def _triangle_key(triangle: np.ndarray, decimals: int = 5) -> tuple[tuple[float, float, float], ...]:
    vertices = np.round(np.asarray(triangle, dtype=np.float64), decimals=decimals)
    return tuple(sorted(tuple(float(v) for v in point) for point in vertices))


def subtract_triangles(combined: StlMesh, subset: StlMesh) -> np.ndarray:
    """Return combined triangles not present in subset, preserving combined order."""
    buckets: dict[tuple[tuple[float, float, float], ...], list[int]] = {}
    for index, triangle in enumerate(combined.triangles):
        buckets.setdefault(_triangle_key(triangle), []).append(index)
    used: set[int] = set()
    for triangle in subset.triangles:
        candidates = buckets.get(_triangle_key(triangle), [])
        candidate = next((index for index in candidates if index not in used), None)
        if candidate is None:
            raise ValueError(f"{subset.name} contains a triangle missing from {combined.name}")
        used.add(candidate)
    if len(used) != subset.triangle_count:
        raise ValueError("duplicate subset triangle could not be matched uniquely")
    return combined.triangles[[i for i in range(combined.triangle_count) if i not in used]].copy()


def reverse_winding(triangles: np.ndarray) -> np.ndarray:
    """Return a copy with reversed winding for the inward-wound rigid shells."""
    return np.asarray(triangles)[:, ::-1, :].copy()


def load_joint_stl_assets(asset_dir: str | Path = ASSET_DIR) -> dict[str, StlMesh | np.ndarray]:
    """Load all three supplied files and derive the assembled material split."""
    root = Path(asset_dir)
    rigid = read_binary_stl(root / "Rigid.stl")
    assembled = read_binary_stl(root / "soft_and_rigid_sections.stl")
    soft = read_binary_stl(root / "Soft.stl")
    assembled_rigid = subtract_triangles(assembled, soft)
    if assembled_rigid.shape[0] != rigid.triangle_count:
        raise ValueError(
            f"assembled rigid split has {assembled_rigid.shape[0]} triangles; "
            f"flat Rigid.stl has {rigid.triangle_count}"
        )
    return {
        "rigid_layout": rigid,
        "assembled": assembled,
        "soft": soft,
        "assembled_rigid_triangles": reverse_winding(assembled_rigid),
    }


def _mesh_metadata(mesh: StlMesh) -> dict:
    low, high = mesh.bounds
    return {
        "file": mesh.path.name,
        "sha256": mesh.sha256,
        "byte_count": mesh.path.stat().st_size,
        "triangle_count": mesh.triangle_count,
        "bounds_source_units": {"min": list(low), "max": list(high)},
    }


def write_manifest(path: str | Path, asset_dir: str | Path = ASSET_DIR) -> dict:
    """Write a deterministic provenance manifest and return its contents."""
    assets = load_joint_stl_assets(asset_dir)
    assembled = assets["assembled"]
    soft = assets["soft"]
    rigid = assets["rigid_layout"]
    manifest = {
        "schema": "exact_joint_stl_reference_v1",
        "source": "user-supplied binary STL files",
        "units_assumption": SOURCE_UNIT_ASSUMPTION,
        "stl_to_m": STL_TO_METRE,
        "display_scale_m_per_source_unit": 0.0006,
        "canonical_fem": {
            "source": "exact_joint/source_joint.json",
            "role": "authoritative JSON midsurface and nonlinear FEM topology",
            "stl_geometry_is_reference_only": True,
        },
        "files": {
            "Rigid.stl": _mesh_metadata(rigid) | {"role": "flat manufacturing layout; not an assembled overlay"},
            "soft_and_rigid_sections.stl": _mesh_metadata(assembled) | {"role": "assembled CAD reference"},
            "Soft.stl": _mesh_metadata(soft) | {"role": "assembled flexible shell subset"},
        },
        "derived_split": {
            "assembled_soft_triangles": soft.triangle_count,
            "assembled_rigid_triangles": int(assets["assembled_rigid_triangles"].shape[0]),
            "match_rule": "sorted triangle vertices quantized to 1e-5 source units",
            "rigid_winding": "reversed for USD display; original bytes preserved",
            "topology_check": "all triangles non-degenerate; assembled soft subset matched uniquely",
        },
        "mapping": {
            "stl_x": "0.2 * json_x",
            "stl_y": "-0.2 * json_z",
            "stl_z": "0.2 * json_y + 28.2716503143",
            "display_transform": "model_x = 0.0006 * cad_x; model_y = 0.0006 * cad_y; model_z = 0.0006 * (cad_z - 6.2716503143)",
            "note": "validated midsurface mapping for the assembled soft shell; display transform only; no FEM thickness rescale",
        },
    }
    path = Path(path)
    path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest

