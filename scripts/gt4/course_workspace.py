"""Explicit paths shared by course evidence/reproduction tools."""
import argparse
from pathlib import Path

def parser_for(description: str | None = None) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    workspace = Path(__file__).resolve().parents[2]
    parser.add_argument('--workspace', type=Path, default=workspace)
    parser.add_argument('--audit-root', type=Path, default=workspace / '.audit/gt4-tracks')
    return parser

def paths(description: str | None = None) -> tuple[Path, Path]:
    args = parser_for(description).parse_args()
    return args.audit_root.resolve(), args.workspace.resolve()
