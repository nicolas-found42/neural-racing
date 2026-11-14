"""Read original-initialized ADLN routes and evaluate their line/arc geometry.

The original 0x46b480 sampler is authoritative. This float64 construction matched
289 independently executed original sampler cases within 0.31 mm; it is not an
EE bit-exact implementation. Initialized coordinates already use physics XY.
Types 1/2/6 are interchangeable driving routes, not road-edge arrays.
"""

import bisect
import itertools
import math
import struct

# Decimal literal retained from the recovered sampler pseudocode. The original
# EE constant is f32 bits 0x40c90fda; this host evaluator uses float64 geometry.
NATIVE_TAU = 6.2831850052


def records(blob: bytes, kind: int = 6) -> list[dict]:
    if blob[:4] != b"ADLN":
        raise ValueError("Only original-initialized 64-byte ADLN is supported")
    base = struct.unpack_from("<I", blob, 4)[0]
    count, pointer = struct.unpack_from("<II", blob, 0x14)
    if not 0 <= kind < count <= 256:
        raise ValueError("ADLN route table range")
    table = pointer - base
    if not 0 <= table <= len(blob) - count * 4:
        raise ValueError("ADLN route table pointer")
    pointer = struct.unpack_from("<I", blob, table + kind * 4)[0]
    if pointer == 0:
        return []
    info = pointer - base
    if not 0 <= info <= len(blob) - 64:
        raise ValueError("ADLN route pointer")
    n = struct.unpack_from("<I", blob, info)[0]
    if not 2 <= n <= (len(blob) - info - 64) // 64:
        raise ValueError("ADLN route record count")
    result = []
    for i in range(n):
        at = info + 64 + i * 64
        x, y, z = struct.unpack_from("<3f", blob, at + 4)
        v, radius = struct.unpack_from("<2f", blob, at + 20)
        angle = struct.unpack_from("<f", blob, at + 36)[0]
        if not all(math.isfinite(f) for f in (x, y, z, v, radius, angle)):
            raise ValueError("Nonfinite initialized ADLN")
        result.append(
            {
                "kind": blob[at],
                "x": x,
                "y": y,
                "z": z,
                "v": v,
                "radius": radius,
                "angle": angle,
                "index": i,
                "material": blob[at + 0x13],
            }
        )
    if any(b["v"] <= a["v"] for a, b in itertools.pairwise(result)):
        raise ValueError("Nonascending native progress is not an ordinary route")
    return result


def sample(rows: list[dict], length: float, native_v: float) -> tuple:
    """Return source XY, interpolated initialized height, unit tangent, segment."""
    t = native_v % length
    i = bisect.bisect_right([a["v"] for a in rows], t) - 1
    a, b = rows[i], rows[(i + 1) % len(rows)]
    av, bv = a["v"], b["v"]
    if i < 0:
        av -= length
    elif i == len(rows) - 1:
        bv += length
    fraction = (t - av) / (bv - av)
    if a["kind"] == 0:
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        x, y = a["x"] + fraction * dx, a["y"] + fraction * dy
    else:
        radius = a["radius"]
        cx = a["x"] - math.cos(a["angle"]) * radius
        cy = a["y"] - math.sin(a["angle"]) * radius
        begin = math.atan2(cx - a["x"], a["y"] - cy)
        end = math.atan2(cx - b["x"], b["y"] - cy)
        if radius > 0 and end < begin:
            end += NATIVE_TAU
        elif radius < 0:
            radius = -radius
            if begin < end:
                begin += NATIVE_TAU
        angle = begin + fraction * (end - begin)
        x, y = cx - math.sin(angle) * radius, cy + math.cos(angle) * radius
        dx = -math.cos(angle) * radius * (end - begin)
        dy = -math.sin(angle) * radius * (end - begin)
    magnitude = math.hypot(dx, dy)
    if magnitude < 1e-9:
        raise ValueError("Degenerate authored tangent")
    return (
        x,
        y,
        a["z"] + fraction * (b["z"] - a["z"]),
        dx / magnitude,
        dy / magnitude,
        a["index"],
    )
