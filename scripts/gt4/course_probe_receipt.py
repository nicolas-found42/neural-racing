#!/usr/bin/env python3
"""Freeze/verify SHA-256 input identities for the independent all-course Rust probe."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT / "crates/neural_racing_core"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("receipt", type=Path)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    if args.verify:
        receipt = json.loads(args.receipt.read_text())
        errors = [name for name, value in receipt["inputs"].items()
                  if not (ROOT / name).is_file() or digest(ROOT / name) != value["sha256"]]
        if errors:
            raise SystemExit("Probe inputs changed: " + ", ".join(errors))
        print(f"Verified {len(receipt['inputs'])} frozen inputs; receipt SHA256 {digest(args.receipt)}")
        return
    paths = set((CORE / "src").rglob("*.rs"))
    paths.update((CORE / "src/gt4/data").glob("*.bin"))
    paths.update((CORE / "src/circuit_data/gt4").glob("*"))
    paths.update(CORE / name for name in ("tests/gt4_courses.rs", "tests/common/mod.rs", "tests/fixtures/gt4_course_lengths.json"))
    paths.add(ROOT / ".audit/gt4-tracks/published-geometry.json")
    paths.update((ROOT / "Cargo.toml", ROOT / "Cargo.lock", CORE / "Cargo.toml"))
    receipt = {
        "schema": 1,
        "rustc": subprocess.check_output(["rustc", "-Vv"], text=True).strip(),
        "scope": "Rust core sources, compiled GT4 road/parameter payloads, independent probe/driver and source-distance fixture; no training",
        "catalog_source": (CORE / "src/circuit_data/gt4_catalog.rs").read_text(),
        "files_source": (CORE / "src/circuit_data/gt4_files.rs").read_text(),
        "inputs": {str(path.relative_to(ROOT)): {"sha256": digest(path), "bytes": path.stat().st_size}
                   for path in sorted(paths) if path.is_file()},
    }
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps(receipt, indent=2) + "\n")
    print(f"Frozen {len(receipt['inputs'])} inputs; receipt SHA256 {digest(args.receipt)}")


if __name__ == "__main__":
    main()
