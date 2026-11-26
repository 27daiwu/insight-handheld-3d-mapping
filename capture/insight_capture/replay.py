"""replay: rosbag2 playback of a session."""
from __future__ import annotations

import os
import subprocess
import sys

from .config import data_root
from .inspect_cmd import session_state
from .session import BAG, read_json, resolve_session, MANIFEST
from . import bagdb


def replay(args) -> int:
    root = data_root(args.root)
    sdir = resolve_session(root, args.session) if args.session != "-" else None
    if sdir is None:
        print("no session found", file=sys.stderr)
        return 2
    bag = sdir / BAG if (sdir / BAG).exists() else sdir
    if not bagdb.db_files(bag):
        print(f"no db3 files in {bag}", file=sys.stderr)
        return 2
    if not (bag / "metadata.yaml").exists():
        print(f"{bag} has no metadata.yaml (unclean recording). Run: insight-capture inspect {sdir} --repair",
              file=sys.stderr)
        return 3
    cmd = ["ros2", "bag", "play", str(bag), "--rate", str(args.rate)]
    if args.clock:
        cmd += ["--clock", "100"]
    if args.loop:
        cmd.append("--loop")
    if args.start_offset:
        cmd += ["--start-offset", str(args.start_offset)]
    if args.topics:
        cmd += ["--topics"] + args.topics
    elif args.roles:
        m = read_json(sdir / MANIFEST, {})
        names = [t["name"] for t in m.get("topics", []) if t.get("role") in args.roles]
        if not names:
            print(f"no topics for roles {args.roles}", file=sys.stderr)
            return 2
        cmd += ["--topics"] + names
    if args.remap_prefix:
        m = read_json(sdir / MANIFEST, {})
        old = (m.get("camera", {}).get("prefix") or "")
        remaps = [f"{t['name']}:={args.remap_prefix}{t['name'][len(old):]}" for t in m.get("topics", [])
                  if t["role"] not in ("tf", "tf_static") and old and t["name"].startswith(old)]
        if remaps:
            cmd += ["--remap"] + remaps
    if not sys.stdin.isatty():
        cmd.append("--disable-keyboard-controls")
    st, why = session_state(sdir)
    if st not in ("complete", "unknown", "recovered"):
        print(f"WARNING: session state is {st}: {why}", file=sys.stderr)
    print("+ " + " ".join(cmd))
    if args.dry_run:
        return 0
    return subprocess.call(cmd)
