"""Profile loading."""
from __future__ import annotations

import os
from pathlib import Path

import yaml

DEFAULT_PROFILE = Path(__file__).resolve().parent.parent / "config" / "topics_v2.1.4.yaml"


def load_profile(path: str | os.PathLike | None = None) -> dict:
    p = Path(path or os.environ.get("INSIGHT_PROFILE") or DEFAULT_PROFILE)
    with open(p, "r", encoding="utf-8") as f:
        prof = yaml.safe_load(f)
    prof["_path"] = str(p)
    return prof


def data_root(cli_value: str | None = None) -> Path:
    return Path(cli_value or os.environ.get("INSIGHT_DATA_ROOT")
                or Path.home() / "insight_data" / "sessions").expanduser()
