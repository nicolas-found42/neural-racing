#!/usr/bin/env python3
"""Batch original relocated RNW contact+BSP/nativeV; no host geometry functions."""

import argparse
import collections
import hashlib
import json
import math
import re
import struct
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "queries", type=Path, help="JSON list of course/index/query XYZ records"
)
parser.add_argument("--audit-root", type=Path, default=ROOT / ".audit/gt4-tracks")
parser.add_argument(
    "--runner",
    type=Path,
    default=ROOT / ".audit/gt4-port/differential/play-build/gt4-runner",
)
parser.add_argument(
    "--elf", type=Path, default=ROOT / ".audit/gt4-handling/extracted/CORE.ps2.elf"
)
parser.add_argument("--output-prefix", default="native-progress")
parser.add_argument(
    "--batch-size", type=int, default=32, choices=range(1, 129), metavar="1..128"
)
parser.add_argument("--timeout", type=int, default=1800)
args = parser.parse_args()
T = args.audit_root.resolve()
R = ROOT
H = T / "native-v"
H.mkdir(parents=True, exist_ok=True)
U = lambda x: struct.pack("<I", x)
expected_target = "e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804"
if hashlib.sha256(args.elf.read_bytes()).hexdigest() != expected_target:
    parser.error(
        "ELF identity differs from the recovered CORE build; its entry addresses are not interchangeable"
    )
base = 0x1800000
tramp = 0x1D00000
owner = 0x1D20000
course = 0x1D24000
qbase = 0x1D40000
obase = 0x1D80000
STOP = 0x1FFF000
load = lambda r, v: [
    0x3C000000 | (r << 16) | (v >> 16),
    0x34000000 | (r << 21) | (r << 16) | (v & 65535),
]
jal = lambda a: 0x0C000000 | (a >> 2)
lw = lambda r, b, o: 0x8C000000 | (b << 21) | (r << 16) | o
sw = lambda r, b, o: 0xAC000000 | (b << 21) | (r << 16) | o
source = args.queries.resolve()
cs = json.loads(source.read_text())
if not isinstance(cs, list) or not cs:
    parser.error("queries must be a nonempty JSON list")
for i, case in enumerate(cs):
    if not re.fullmatch(r"[A-Za-z0-9_-]+", str(case.get("course", ""))):
        parser.error(f"query {i} has an invalid course resource ID")
    point = case.get("query")
    if (
        not isinstance(point, list)
        or len(point) != 3
        or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in point)
    ):
        parser.error(f"query {i} needs three finite native XYZ values")
grouped = collections.defaultdict(list)
for i, c in enumerate(cs):
    grouped[c["course"]].append(i)
prefix = args.output_prefix
limit = args.batch_size
if Path(prefix).name != prefix:
    parser.error("output prefix must be a filename component")
batches = []
sources = {}
for name, ids in grouped.items():
    for begin in range(0, len(ids), limit):
        batches.append((name, ids[begin : begin + limit]))
inputs = H / f"{prefix}-inputs.bin"
with inputs.open("wb") as f:
    f.write(b"GT4RUN1\0" + U(len(batches)))
    for name, ids in batches:
        meta = json.loads((T / "decoded" / f"{name}.json").read_text())
        rw = (T / meta["runway_file"]).read_bytes()
        if rw[:4] != b"RNW4" or len(rw) > tramp - base:
            parser.error(
                f"{name} is outside the admitted original runway memory layout"
            )
        sources[name] = {
            "path": meta["runway_file"],
            "sha256": hashlib.sha256(rw).hexdigest(),
            "query_count": len(grouped[name]),
        }
        words = load(4, base) + [jal(0x3E5818), 0]
        points = bytearray(0x34 * len(ids))
        for n, i in enumerate(ids):
            q = qbase + n * 0x34
            o = obase + n * 16
            struct.pack_into("<3f", points, n * 0x34, *cs[i]["query"])
            struct.pack_into("<I", points, n * 0x34 + 0xC, 0xFFFFFFFF)
            words += (
                load(4, owner)
                + load(5, q)
                + [jal(0x395760), 0]
                + load(4, base)
                + load(5, q)
                + load(6, q + 0xC)
                + [jal(0x3E6858), 0]
                + load(10, o)
                + [0xE4000000 | (10 << 21)]
                + load(11, q)
                + [
                    lw(12, 11, 0xC),
                    sw(12, 10, 4),
                    lw(12, 11, 0x10),
                    sw(12, 10, 8),
                    lw(12, 11, 0x30),
                    sw(12, 10, 12),
                ]
            )
        words += [0x02000008, 0]
        code = b"".join(U(w) for w in words)
        assert len(code) < owner - tramp
        own = bytearray(8)
        struct.pack_into("<I", own, 4, course)
        co = bytearray(0xE0)
        struct.pack_into("<I", co, 0xDC, base)
        patches = [
            (base, rw),
            (tramp, code),
            (owner, own),
            (course, co),
            (qbase, points),
        ]
        f.write(
            U(tramp)
            + U(STOP)
            + U(1)
            + U(16)
            + struct.pack("<Q", STOP)
            + U(0)
            + U(len(patches))
        )
        for a, b in patches:
            f.write(U(a) + U(len(b)) + b)
        f.write(
            U(len(ids) * 4)
            + b"".join(
                U(obase + n * 16 + j * 4) for n in range(len(ids)) for j in range(4)
            )
        )
runner = args.runner.resolve()
target = args.elf.resolve()
start = time.monotonic()
with (
    (H / f"{prefix}-stdout.txt").open("w") as out,
    (H / f"{prefix}-stderr.txt").open("w") as err,
):
    p = subprocess.run(
        [str(runner), str(target), str(inputs)],
        stdout=out,
        stderr=err,
        check=False,
        timeout=args.timeout,
    )
if p.returncode:
    raise RuntimeError(
        "runner return "
        + str(p.returncode)
        + " stderr "
        + (H / f"{prefix}-stderr.txt").read_text()[-3000:]
    )
lines = (H / f"{prefix}-stdout.txt").read_text().splitlines()
assert len(lines) == len(batches)
cycles = []
for k, ((name, ids), line) in enumerate(zip(batches, lines)):
    cells = line.split()
    assert int(cells[0]) == k and int(cells[1], 16) == STOP
    values = cells[5:]
    assert len(values) == len(ids) * 4
    cycles.append(int(cells[4]))
    for n, i in enumerate(ids):
        v, h, y, flags = values[n * 4 : n * 4 + 4]
        c = cs[i]
        c.update(
            native_v_bits=v,
            native_v=struct.unpack("<f", U(int(v, 16)))[0],
            native_hint=h,
            native_hit_height=struct.unpack("<f", U(int(y, 16)))[0],
            native_contact_flags_word=flags,
            native_query_succeeded=(int(flags, 16) & 255) == 0,
            batch=k,
        )
summary = collections.defaultdict(lambda: {"queries": 0, "successful": 0})
for c in cs:
    summary[c["course"]]["queries"] += 1
    summary[c["course"]]["successful"] += int(c["native_query_succeeded"])
with inputs.open("rb") as input_stream:
    input_sha256 = hashlib.file_digest(input_stream, "sha256").hexdigest()
report = {
    "authority": "unchanged original3e5818→395760→3e6858 on upstream Play EE; actualBSP search/traverse/checkHit andclusterlimitedquadprojection; no host geometry callbacks",
    "target_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
    "runner_sha256": hashlib.sha256(runner.read_bytes()).hexdigest(),
    "input_sha256": input_sha256,
    "query_source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
    "emulator_commit": "1f27232772e484bfb79364b751b84f41f4c87481",
    "query_count": len(cs),
    "batch_count": len(batches),
    "batch_size": limit,
    "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    "elapsed_seconds": time.monotonic() - start,
    "max_batch_cycles": max(cycles),
    "sources": sources,
    "course_summary": summary,
    "cases": cs,
}
(H / f"{prefix}-results.json").write_text(json.dumps(report, indent=2) + "\n")
print(
    "queries",
    len(cs),
    "courses",
    len(sources),
    "batches",
    len(batches),
    "successful",
    sum(c["native_query_succeeded"] for c in cs),
    "elapsed",
    report["elapsed_seconds"],
    "maxcycles",
    max(cycles),
)
print(
    "failedcourses",
    {k: v for k, v in summary.items() if v["successful"] != v["queries"]},
)
