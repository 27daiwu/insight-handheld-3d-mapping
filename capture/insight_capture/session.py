"""Session directory layout, state file and helpers."""
from __future__ import annotations

import json
import os
import re
import socket
import time
from datetime import datetime, timezone
from pathlib import Path

STATE = "session.json"       # live state / heartbeat (supervisor owns it)
MANIFEST = "manifest.json"
PLAN = "plan.json"           # what the supervisor was asked to record
BAG = "bag"
LOG = "recorder.log"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_json(path: Path, obj) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False, default=str)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def read_json(path: Path, default=None):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def new_session(root: Path, name: str | None = None) -> tuple[str, Path]:
    root.mkdir(parents=True, exist_ok=True)
    base = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = re.sub(r"[^A-Za-z0-9_.-]+", "-", name) if name else socket.gethostname()
    sid = f"{base}_{suffix}"
    d = root / sid
    n = 1
    while d.exists():          # never reuse a session directory
        n += 1
        d = root / f"{sid}-{n}"
    sid = d.name
    d.mkdir()
    return sid, d


def pid_alive(pid: int | None) -> bool:
    if not pid:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    # zombie check
    try:
        st = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
        return st != "Z"
    except OSError:
        return True


def active_marker(root: Path) -> Path:
    return root / "ACTIVE"


def active_session(root: Path) -> Path | None:
    m = active_marker(root)
    try:
        d = Path(m.read_text().strip())
    except OSError:
        return None
    st = read_json(d / STATE, {})
    if st.get("status") == "recording" and pid_alive(st.get("supervisor_pid")):
        return d
    return None


def resolve_session(root: Path, ref: str | None) -> Path | None:
    """ref: path, session id, 'latest' or None (active, else latest)."""
    if ref and ref != "latest":
        p = Path(ref)
        if p.is_dir():
            return p
        if (root / ref).is_dir():
            return root / ref
        hits = sorted(d for d in root.glob(f"*{ref}*") if d.is_dir()) if root.exists() else []
        return hits[-1] if hits else None
    if ref is None:
        a = active_session(root)
        if a:
            return a
    if not root.exists():
        return None
    ds = sorted(d for d in root.iterdir() if d.is_dir() and ((d / MANIFEST).exists() or (d / PLAN).exists()))
    return ds[-1] if ds else None


def bag_size(bag: Path) -> int:
    total = 0
    if bag.exists():
        for p in bag.iterdir():
            try:
                total += p.stat().st_size
            except OSError:
                pass
    return total


def now_ns() -> int:
    return time.time_ns()
