"""Bind aligned candidate stations/edges to original EE contact/getV outputs."""

import argparse
import hashlib
import json
import math
import struct
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--mesh-dir", type=Path, required=True)
    parser.add_argument("--runway-dir", type=Path, required=True)
    parser.add_argument(
        "--results",
        type=Path,
        action="append",
        required=True,
        help="Original results, later files override changed query entries",
    )
    args = parser.parse_args()
    S = args.candidates.resolve()
    O = args.out.resolve()
    O.mkdir(parents=True, exist_ok=True)
    sources = [p.resolve() for p in args.results]
    native = {}
    for p in sources:
        for q in json.loads(p.read_text())["cases"]:
            native[q["course"], q["index"]] = q
    meta = {q["id"]: q for q in json.loads(args.metadata.read_text())["courses"]}
    proof = []
    fallback = []
    for p in sorted(S.glob("*.stations")):
        ident = p.stem
        data = p.read_bytes()
        edge = (S / (ident + ".stations.edges")).read_bytes()
        n = struct.unpack_from("<I", data)[0]
        rows = [list(struct.unpack_from("<6fII", data, 4 + i * 32)) for i in range(n)]
        ee = [edge[4 + i * 16 : 4 + (i + 1) * 16] for i in range(n)]
        L = struct.unpack_from(
            "<f", (args.runway_dir / f"{ident}.rnw").read_bytes(), 20
        )[0]
        vv = []
        height_updates = 0
        material_updates = 0
        mesh = (args.mesh_dir / f"{ident}.bin").read_bytes()
        nv, nt = struct.unpack_from("<II", mesh)
        for i, q in enumerate(rows):
            hit = native[ident, i]
            assert hit["native_query_succeeded"] and hit["query"] == [
                q[1],
                q[3],
                -q[2],
            ], (ident, i, "query mismatch")
            a = q[0]
            v = hit["native_v"]
            zero = v == 0 and int(hit["native_hint"], 16) >> 16 == 0
            vv.append(None if zero else a + (v - a + L * 0.5) % L - L * 0.5)
            h = hit["native_hit_height"]
            height_updates += h != q[3]
            q[3] = h
            tri = int(hit["native_hint"], 16) & 65535
            assert tri < nt
            mat = mesh[8 + nv * 16 + tri * 12 + 6] & 31
            material_updates += q[7] != mat
            q[7] = mat
        i = 0
        while i < n:
            if vv[i] is not None:
                i += 1
                continue
            start = i
            while i < n and vv[i] is None:
                i += 1
            stop = i
            assert start > 0 and stop < n
            lengths = [0.0]
            for k in range(start, stop + 1):
                lengths.append(lengths[-1] + math.dist(rows[k - 1][1:4], rows[k][1:4]))
            for k in range(start, stop):
                t = lengths[k - start + 1] / lengths[-1]
                vv[k] = vv[start - 1] + t * (vv[stop] - vv[start - 1])
                fallback.append(
                    {
                        "course": ident,
                        "index": k,
                        "raw_bits": native[ident, k]["native_v_bits"],
                        "hint": native[ident, k]["native_hint"],
                        "derived_native_v": vv[k],
                        "method": "physical-distance interpolation between proven original-V neighbors after original cluster-zero fallback",
                        "neighbors": [start - 1, stop],
                    }
                )
        assert all(vv[i] > vv[i - 1] + 1e-5 for i in range(1, n)), (
            ident,
            "nonascending original V",
        )
        full = (
            abs(rows[-1][0] - rows[0][0] - L) < 0.01
            and math.dist(rows[0][1:4], rows[-1][1:4]) < 0.001
        )
        # Race section topology does not change the underlying RNW loop.
        # Normalize the original positive native V and remove paired duplicates
        # for full source loops, including the dedicated testcourse sprints.
        drop = (
            full
            and meta[ident]["source_topology"] == 0
            and meta[ident]["topology"] != "Area"
        )
        chosen = list(range(n - 1 if drop else n))
        out = []
        for k in chosen:
            v = vv[k] % L if drop else vv[k]
            r = rows[k]
            r[0] = v
            out.append((v, k, r))
        out.sort(key=lambda a: a[0])
        raw = bytearray(struct.pack("<I", len(out)))
        ed = bytearray(struct.pack("<I", len(out)))
        last = -math.inf
        for v, k, q in out:
            f = struct.unpack("<f", struct.pack("<f", v))[0]
            assert f > last + 1e-5, (ident, k, f, last)
            last = f
            raw += struct.pack("<6fII", *q)
            ed += ee[k]
        (O / (ident + ".stations")).write_bytes(raw)
        (O / (ident + ".edges")).write_bytes(ed)
        proof.append(
            {
                "id": ident,
                "before_count": n,
                "after_count": len(out),
                "closed_duplicate_removed": drop,
                "source_stations_sha256": hashlib.sha256(data).hexdigest(),
                "stations_sha256": hashlib.sha256(raw).hexdigest(),
                "edges_sha256": hashlib.sha256(ed).hexdigest(),
                "native_height_updates": height_updates,
                "native_material_updates": material_updates,
                "xy_unchanged": True,
                "source_edge_positions_unchanged": True,
            }
        )
    receipt = {
        "status": "original-native-bound geometry; actual-car acceptance pending",
        "sources": {
            str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources
        },
        "courses": proof,
        "fallbacks": fallback,
        "rule": "Original EE native V and hit height/material take precedence; Explicit cluster-zero V fallbacks interpolate between proven native neighbors. Original XY and paired source edge coordinates are unchanged. Closed duplicate endpoints removed with paired edges; full cyclic underlying source pairs normalized even for open race sections.",
    }
    (O / "freeze.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(
        len(proof),
        "courses",
        sum(q["after_count"] for q in proof),
        "stations",
        len(fallback),
        "fallbacks",
        "sha",
        hashlib.sha256((O / "freeze.json").read_bytes()).hexdigest(),
    )


if __name__ == "__main__":
    main()
