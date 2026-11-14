"""Build auditable candidate navigation/edge payloads; never installs game assets.

Inputs are original-initialized ADLN and the retained RNW/manual-navigation
baseline. Native source mesh and boundaries stay unchanged. Ground cross-section
edges describe physical source partitions, not painted racing-strip boundaries.
"""

import argparse
import concurrent.futures
import hashlib
import itertools
import json
import math
import struct
import subprocess
from pathlib import Path

from adln_route import records, sample


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read_stations(path):
    data = path.read_bytes()
    n = struct.unpack_from("<I", data)[0]
    if len(data) != 4 + n * 32 or n < 2:
        raise ValueError(f"Invalid stations: {path}")
    return data, [struct.unpack_from("<6fII", data, 4 + i * 32) for i in range(n)]


def gate(x, y, height, dx, dy, v, index):
    magnitude = math.hypot(dx, dy)
    if magnitude < 1e-9:
        raise ValueError("Degenerate navigation tangent")
    # Cross-section positive axis is core LEFT [-ty,+tx], converted to RNW X/Z.
    rx, rz = -dy / magnitude, -dx / magnitude
    return struct.pack(
        "<10fI",
        x - rx,
        height,
        -y - rz,
        x,
        height,
        -y,
        x + rx,
        height,
        -y + rz,
        v,
        index,
    )


def build_one(course, args, initialization):
    ident = course["id"]
    baseline_path = args.baseline / f"{ident}.stations"
    old, baseline = (
        read_stations(baseline_path) if baseline_path.exists() else (None, None)
    )
    rnw = args.audit_root / "decoded/runways" / f"{ident}.rnw"
    raw = rnw.read_bytes()
    length = struct.unpack_from("<f", raw, 20)[0]
    source = args.initialized_adln / f"{ident}-initialized-adln.bin"
    rows = []
    reason = "original RNW/manual local navigation baseline"
    if (
        course["topology"] != "Area"
        and ident not in args.retain_navigation
        and source.exists()
    ):
        initialized = source.read_bytes()
        receipt = initialization[ident]
        if sha(initialized) != receipt["initialized_adln_sha256"]:
            raise ValueError(f"Initialized source hash mismatch: {ident}")
        if sha(raw) != receipt["original_rnw_sha256"]:
            raise ValueError(f"Original RNW hash mismatch: {ident}")
        try:
            rows = records(initialized, 6)
        except ValueError as exc:
            if "Nonascending" not in str(exc):
                raise
            reason = (
                "native route6 has nonascending progress; retained local navigation"
            )
    if rows:
        start, end = 0.0, length
        count = math.ceil((end - start) / 2) + 1
        gates = bytearray(struct.pack("<I", count))
        for i in range(count):
            v = start + (end - start) * i / (count - 1)
            x, y, h, dx, dy, segment = sample(rows, length, v)
            gates += gate(x, y, h, dx, dy, v, segment)
        preferred = int(
            sum(q["material"] in (2, 3, 4, 6) for q in rows) > len(rows) / 2
        )
        reason = "original-initialized authored ADLN route6 line/arc sampler"
    else:
        if baseline is None:
            raise ValueError(f"Missing explicit RNW navigation fallback: {ident}")
        gates = bytearray(struct.pack("<I", len(baseline)))
        for i, q in enumerate(baseline):
            a, b = baseline[max(0, i - 1)], baseline[min(len(baseline) - 1, i + 1)]
            gates += gate(q[1], q[2], q[3], b[1] - a[1], b[2] - a[2], q[0], q[6])
        preferred = int(sum(q[7] in (2, 3, 4, 6) for q in baseline) > len(baseline) / 2)
    gate_path = args.out / f"{ident}.gates"
    gate_path.write_bytes(gates)
    target = args.out / f"{ident}.stations"
    boundary = (args.boundaries or args.baseline) / f"{ident}.boundaries"
    mesh = args.audit_root / "decoded/meshes" / f"{ident}.bin"
    run = subprocess.run(
        [
            str(args.corridor_executable),
            str(mesh),
            str(gate_path),
            str(target),
            str(boundary),
            str(preferred),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    data, result = read_stations(target)
    edge_path = target.with_suffix(".stations.edges")
    edge_data = edge_path.read_bytes()
    if len(edge_data) != 4 + len(result) * 16:
        raise ValueError(f"Edge alignment mismatch: {ident}")
    if not all(
        all(math.isfinite(x) for x in q[:6]) and min(q[4:6]) > 0 for q in result
    ):
        raise ValueError(f"Missing or inverted source cross-section: {ident}")
    queries = [
        {
            "course": ident,
            "index": i,
            "query": [q[1], q[3], -q[2]],
            "assigned_native_v": q[0],
        }
        for i, q in enumerate(result)
    ]
    return {
        "id": ident,
        "count": len(result),
        "navigation": reason,
        "baseline_sha256": sha(old) if old is not None else None,
        "initialized_adln_sha256": sha(source.read_bytes()) if rows else None,
        "rnw_sha256": sha(raw),
        "mesh_sha256": sha(mesh.read_bytes()),
        "boundaries_sha256": sha(boundary.read_bytes()),
        "stations_sha256": sha(data),
        "edges_sha256": sha(edge_data),
        "minimum_width": min(q[4] + q[5] for q in result),
        "maximum_planar_step": max(
            math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in itertools.pairwise(result)
        ),
        "native_surface_class": preferred,
        "measurement_stdout": run.stdout.strip(),
    }, queries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-root", type=Path, required=True)
    parser.add_argument("--initialized-adln", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--boundaries", type=Path)
    parser.add_argument(
        "--retain-navigation",
        action="append",
        default=[],
        help="Explicit proven RNW baseline override (recorded in receipt)",
    )
    parser.add_argument("--corridor-executable", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=8)
    args = parser.parse_args()
    for name in (
        "audit_root",
        "initialized_adln",
        "baseline",
        "corridor_executable",
        "out",
    ):
        setattr(args, name, getattr(args, name).resolve())
    if args.boundaries is not None:
        args.boundaries = args.boundaries.resolve()
    args.out.mkdir(parents=True, exist_ok=True)
    courses = json.loads(
        (args.audit_root / "decoded/course-metadata.json").read_text()
    )["courses"]
    courses = [q for q in courses if q["available"]]
    init_receipt = args.initialized_adln / "native-adln-initialization.json"
    if not init_receipt.exists():
        init_receipt = args.initialized_adln / "all-courses-initialization.json"
    initialization = {
        q["course"]: q for q in json.loads(init_receipt.read_text())["courses"]
    }
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        outputs = list(pool.map(lambda q: build_one(q, args, initialization), courses))
    queries = [q for _, batch in outputs for q in batch]
    query_data = (json.dumps(queries, separators=(",", ":")) + "\n").encode()
    (args.out / "native-query-inputs.json").write_bytes(query_data)
    receipt = {
        "status": "candidate-only; original contact verification and actual-car audit still required",
        "count": len(queries),
        "courses": [row for row, _ in outputs],
        "navigation_clearance": {
            "selected_m3_mirror_width_m": 1.914,
            "margin_each_side_m": 0.10,
        },
        "boundary_rule": "Original directed blockers from the same height-continuous mesh deck",
        "surface_rule": "Dominant source native surface class; physical partitions are not painted edges",
        "query_sha256": sha(query_data),
        "generator_sha256": sha(Path(__file__).read_bytes()),
        "corridor_executable_sha256": sha(args.corridor_executable.read_bytes()),
    }
    (args.out / "freeze.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(
        f"{len(outputs)} courses; {len(queries)} candidate stations; no runtime files changed"
    )


if __name__ == "__main__":
    main()
