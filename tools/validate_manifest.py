#!/usr/bin/env python3
"""Validate a Session manifest against schemas/session_manifest.schema.json.

Usage: tools/validate_manifest.py <session_dir | manifest.json>
Requires: python3-jsonschema (not installed by this repo).
"""
import json
import pathlib
import sys

SCHEMA = pathlib.Path(__file__).resolve().parent.parent / "schemas" / "session_manifest.schema.json"


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    import jsonschema  # imported lazily so --help works without it

    p = pathlib.Path(sys.argv[1])
    manifest = p / "manifest.json" if p.is_dir() else p
    schema = json.loads(SCHEMA.read_text())
    jsonschema.Draft202012Validator.check_schema(schema)
    errors = sorted(
        jsonschema.Draft202012Validator(schema).iter_errors(json.loads(manifest.read_text())),
        key=lambda e: list(e.path),
    )
    for e in errors:
        print(f"{'/'.join(map(str, e.path)) or '<root>'}: {e.message}")
    if p.is_dir():
        for sub in ("bag", "logs", "calibration"):
            if not (p / sub).is_dir():
                print(f"missing directory: {sub}/")
                errors.append(sub)
    print("VALID" if not errors else f"INVALID ({len(errors)} problems)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
