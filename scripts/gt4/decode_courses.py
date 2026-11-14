#!/usr/bin/env python3
"""Bounded original GT4 RNW4 geometry decoder. No smoothing, scale fitting or roadwidth guesses."""

import argparse
import collections
import gzip
import hashlib
import itertools
import json
import math
import struct
from pathlib import Path


class Reader:
    def __init__(self, b):
        self.b = b

    def get(self, fmt, o):
        n = struct.calcsize("<" + fmt)
        if o < 0 or o + n > len(self.b):
            raise ValueError(f"out of bounds {o:x}+{n:x}")
        return struct.unpack_from("<" + fmt, self.b, o)

    def span(self, o, n, stride):
        if (
            not 0 <= n <= 1000000
            or not 0 <= o <= len(self.b)
            or n * stride > len(self.b) - o
        ):
            raise ValueError("invalid array")
        return range(n)

    def rows(self, fmt, o, n, stride):
        return [list(self.get(fmt, o + i * stride)) for i in self.span(o, n, stride)]


def finite(xs):
    if any(not math.isfinite(x) for x in xs):
        raise ValueError("nonfinite source coordinate")


def decode_ad(path):
    b = path.read_bytes()
    r = Reader(b)
    base = r.get("I", 4)[0]
    if b[base : base + 4] != b"ADLN":
        raise ValueError("ADLN missing")
    n, table = r.get("II", base + 0x14)
    infos = []
    for i in r.span(base + table, n, 4):
        p = base + r.get("I", base + table + i * 4)[0]
        count = r.get("I", p)[0]
        rows = r.rows("I3fhh5f", p + 0x40, count, 0x40)
        for row in rows:
            finite(row[1:4])
        infos.append(
            {
                "source_offset": p,
                "attack_positions": [v[1:4] for v in rows],
                "raw_attack_records": rows,
            }
        )
    return {
        "sha256": hashlib.sha256(b).hexdigest(),
        "adln_offset": base,
        "lines": infos,
    }


def decode_course(path, out):
    b = path.read_bytes()
    outer = Reader(b)
    off = outer.get("I", 0xDC)[0]
    if off < 0x100 or b[off : off + 4] != b"RNW4":
        raise ValueError("DC runway pointer does not bind RNW4")
    size = outer.get("I", off + 8)[0]
    r = Reader(b[off : off + size])
    _magic, relocation_state, declared, flags, version = r.get("5I", 0)
    if relocation_state != 0:
        raise ValueError("expected unrelocated RNW pointers")
    if declared != size:
        raise ValueError("size mismatch")
    track, start, goal = r.get("3f", 0x14)
    finite([track, start, goal])
    bounds = r.rows("3f", 0x20, 2, 12)
    counts = dict(
        zip(
            ["checkpoint_lists", "grids", "checkpoints", "checkpoint_lookup"],
            r.get("4H", 0x38),
        )
    )
    counts.update(
        dict(
            zip(
                [
                    "gadgets",
                    "vertices",
                    "triangles",
                    "clusters",
                    "tree_depth",
                    "boundary_vertices",
                    "unknown",
                    "pitstops",
                    "sections",
                ],
                r.get("9H", 0x44),
            )
        )
    )
    ptrs = dict(
        zip(
            [
                "checkpoint_lists",
                "grids",
                "checkpoints",
                "checkpoint_lookup",
                "lights",
                "unknown",
                "gadgets",
                "vertices",
                "triangles",
                "clusters",
                "tree",
                "boundary_vertices",
                "boundary_index_count",
                "boundary_indices",
                "pitstops",
                "tire_flags",
                "unknown_98",
                "sections",
            ],
            r.get("18I", 0x58),
        )
    )
    cps = r.rows("10f", ptrs["checkpoints"], counts["checkpoints"], 40)
    for v in cps:
        finite(v)
    verts = r.rows("3fhBB", ptrs["vertices"], counts["vertices"], 16)
    for v in verts:
        finite(v[:3])
    tris = r.rows("3HBBI", ptrs["triangles"], counts["triangles"], 12)
    if any(max(v[:3]) >= len(verts) for v in tris):
        raise ValueError("invalid vertex index")
    surfaces = collections.Counter(v[3] & 31 for v in tris)
    sectors = []
    for i in r.span(ptrs["checkpoint_lists"], counts["checkpoint_lists"], 8):
        n, p = r.get("II", ptrs["checkpoint_lists"] + i * 8)
        sectors.append(list(r.get(str(n) + "f", p)) if n else [])
    sections = []
    for v in r.rows("2f4H4I", ptrs["sections"], counts["sections"], 32):
        sections.append(
            {
                "start_v": v[0],
                "goal_v": v[1],
                "checkpoint_count": v[2],
                "unknown": v[3],
                "gadget_count": v[4],
                "stop_count": v[5],
                "stop_pointer": v[6],
                "checkpoints_pointer": v[7],
                "grid_pointer": v[8],
                "gadget_pointer": v[9],
            }
        )
    points = [
        {
            "source_index": i,
            "left_validation_gate": v[:3],
            "middle": v[3:6],
            "right_validation_gate": v[6:9],
            "track_v": v[9],
        }
        for i, v in enumerate(cps)
    ]
    sorted_ix = sorted(range(len(cps)), key=lambda i: cps[i][9])
    descents = sum(cps[i][9] < cps[i - 1][9] for i in range(1, len(cps)))
    polyline = [cps[i][3:6] for i in sorted_ix]
    dist = lambda a, b: math.dist(a, b)
    measured = sum(dist(a, c) for a, c in itertools.pairwise(polyline))
    gap = dist(polyline[-1], polyline[0]) if len(polyline) > 1 else 0
    mesh = {
        "format": "GT4 RNW4 original road mesh",
        "source_course_sha256": hashlib.sha256(b).hexdigest(),
        "source_runway_offset": off,
        "vertices_layout": [
            "x",
            "y_elevation",
            "z",
            "raw_index",
            "shade_byte",
            "grip_byte",
        ],
        "vertices": verts,
        "triangles_layout": [
            "v0",
            "v1",
            "v2",
            "surface_type_and_bits",
            "surface_flags",
            "raw_u32",
        ],
        "triangles": tris,
        "surface_type_mask": 31,
        "boundary_vertices": r.rows(
            "3fI", ptrs["boundary_vertices"], counts["boundary_vertices"], 16
        ),
        "boundary_indices": list(
            r.get(str(ptrs["boundary_index_count"]) + "h", ptrs["boundary_indices"])
        )
        if ptrs["boundary_index_count"]
        else [],
    }
    meshfile = out / "meshes" / (path.name + ".json.gz")
    meshfile.parent.mkdir(parents=True, exist_ok=True)
    raw = json.dumps(mesh, separators=(",", ":")).encode()
    meshfile.write_bytes(gzip.compress(raw, mtime=0))
    rawmesh = out / "meshes" / (path.name + ".bin")
    rawbytes = (
        struct.pack("<II", counts["vertices"], counts["triangles"])
        + r.b[ptrs["vertices"] : ptrs["vertices"] + counts["vertices"] * 16]
        + r.b[ptrs["triangles"] : ptrs["triangles"] + counts["triangles"] * 12]
    )
    rawmesh.write_bytes(rawbytes)
    rwyfile = out / "runways" / (path.name + ".rnw")
    rwyfile.parent.mkdir(parents=True, exist_ok=True)
    rwyfile.write_bytes(r.b)
    info = {
        "id": path.name,
        "resource_path": "crs/" + path.name,
        "source_sha256": mesh["source_course_sha256"],
        "source_bytes": len(b),
        "status": "decoded_original_geometry",
        "runway": {
            "source_offset": off,
            "source_bytes": size,
            "sha256": hashlib.sha256(r.b).hexdigest(),
            "version": version,
            "flags": flags,
            "track_v_m": track,
            "start_v": start,
            "goal_v": goal,
            "bounds": bounds,
            "counts": counts,
            "pointers": ptrs,
        },
        "checkpoints": points,
        "checkpoint_progress_order": sorted_ix,
        "checkpoint_source_v_descents": descents,
        "checkpoint_middle_open_length_m": measured,
        "checkpoint_middle_closure_gap_m": gap,
        "grids": r.rows("4f", ptrs["grids"], counts["grids"], 16),
        "pitstops": r.rows("4f", ptrs["pitstops"], counts["pitstops"], 16),
        "sectors": sectors,
        "sections": sections,
        "surface_triangle_counts": dict(sorted(surfaces.items())),
        "mesh_file": str(meshfile.relative_to(out.parent)),
        "mesh_sha256": hashlib.sha256(meshfile.read_bytes()).hexdigest(),
        "raw_mesh_file": str(rawmesh.relative_to(out.parent)),
        "raw_mesh_sha256": hashlib.sha256(rawbytes).hexdigest(),
        "runway_file": str(rwyfile.relative_to(out.parent)),
        "gate_widths_m": [math.dist(v[:3], v[6:9]) for v in cps],
        "warnings": [
            "Checkpoint gates are NOT road widths.",
            "Topology and labels require COURSE database/section binding.",
            "Surface names are not inferred from numeric IDs.",
        ],
    }
    ad = path.with_name(path.name + ".ad")
    if ad.exists():
        try:
            info["autodrive"] = dict(status="decoded", **decode_ad(ad))
        except ValueError as e:
            info["autodrive"] = {
                "status": "no_supported_adln",
                "sha256": hashlib.sha256(ad.read_bytes()).hexdigest(),
                "reason": str(e),
            }
    return info


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("resources", type=Path, help="Extracted original crs directory")
    parser.add_argument(
        "output", type=Path, help="Destination for bounded RNW geometry and provenance"
    )
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    items = []
    for p in sorted(args.resources.iterdir()):
        if p.suffix:
            continue
        try:
            info = decode_course(p, out)
        except (ValueError, struct.error, IndexError, OSError) as e:
            info = {
                "id": p.name,
                "resource_path": "crs/" + p.name,
                "status": "unsupported",
                "error": str(e),
            }
        (out / (p.name + ".json")).write_text(json.dumps(info, indent=2) + "\n")
        items.append(info)
    summary = {
        "schema": 1,
        "source": "unchanged user GT4.VOL top-level crs resources",
        "courses": [
            {
                k: v
                for k, v in d.items()
                if k
                not in [
                    "checkpoints",
                    "checkpoint_progress_order",
                    "autodrive",
                    "gate_widths_m",
                    "sectors",
                ]
            }
            for d in items
        ],
    }
    (out / "course-inventory.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(collections.Counter(d["status"] for d in items))
    print(
        [
            (d["id"], d.get("error"))
            for d in items
            if d["status"] != "decoded_original_geometry"
        ]
    )
