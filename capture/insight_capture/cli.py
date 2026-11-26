"""insight-capture command line."""
from __future__ import annotations

import argparse
import json
import sys

from . import __version__


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="insight-capture", description="Insight 9 capture tooling (ROS 2 Humble, rosbag2)")
    p.add_argument("--version", action="version", version=__version__)
    p.add_argument("--root", help="sessions root dir (default $INSIGHT_DATA_ROOT or ~/insight_data/sessions)")
    p.add_argument("--profile", help="topic profile yaml (default capture/config/topics_v2.1.4.yaml)")
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp):
        sp.add_argument("--prefix", help="camera topic prefix, e.g. /insight9 (default: auto-discover)")
        sp.add_argument("--firmware", help="firmware version string (no ROS interface known to read it)")
        sp.add_argument("--expect", action="append", metavar="ROLE=HZ",
                        help="configured rate to compare against, e.g. imu=200 (repeatable)")
        sp.add_argument("--min-free-gb", type=float)
        sp.add_argument("--camera-api", help="camera web API base (read-only GETs), default http://169.254.10.1; 'off' disables")
        sp.add_argument("--image-mode", choices=["raw", "compressed", "both"], default="raw")

    s = sub.add_parser("preflight", help="pre-capture checks (read-only)")
    common(s)
    s.add_argument("--probe-seconds", type=float, default=5.0, help="passive measurement window")
    s.add_argument("--clock-seconds", type=float, default=35.0,
                   help="host wall-clock step check window (>= 33 s catches a 32 s timesyncd cycle; 0 = skip)")
    s.add_argument("--json", action="store_true")

    s = sub.add_parser("record", help="start a new recording session")
    common(s)
    s.add_argument("--name", help="session name suffix")
    s.add_argument("--duration", type=float, help="auto-stop after N seconds")
    s.add_argument("--foreground", action="store_true", help="stay attached until stop/duration")
    s.add_argument("--probe-seconds", type=float, default=3.0)
    s.add_argument("--skip-preflight", action="store_true", help="skip the rate probe (discovery still runs)")
    s.add_argument("--include-extra", action="store_true", help="also record other topics under the camera prefix")
    s.add_argument("--allow-missing", action="store_true", help="record even if required topics are missing")
    s.add_argument("--force", action="store_true", help="record despite preflight FAIL")

    s = sub.add_parser("stop", help="stop the active recording gracefully")
    s.add_argument("session", nargs="?")
    s.add_argument("--timeout", type=float, default=90)

    s = sub.add_parser("inspect", help="check a session/bag (also after crashes)")
    s.add_argument("session", nargs="?", help="session id/path, 'latest' (default: active or latest)")
    s.add_argument("--json", action="store_true")
    s.add_argument("--repair", action="store_true", help="reindex bag metadata and finalize manifest after a crash")
    s.add_argument("--no-tf", action="store_true")
    s.add_argument("--no-diagnose", action="store_true", help="skip continuity/sync diagnostics")
    s.add_argument("--diagnose-out", help="write diagnostics JSON here (default: <session>/diagnostics.json)")

    s = sub.add_parser("replay", help="replay a session with ros2 bag play")
    s.add_argument("session", nargs="?")
    s.add_argument("-r", "--rate", type=float, default=1.0)
    s.add_argument("--loop", action="store_true")
    s.add_argument("--clock", action="store_true", help="publish /clock from bag time")
    s.add_argument("--start-offset", type=float)
    s.add_argument("--topics", nargs="+")
    s.add_argument("--roles", nargs="+", help="replay only these roles (rgb imu vio ...)")
    s.add_argument("--remap-prefix", help="republish camera topics under another prefix")
    s.add_argument("--dry-run", action="store_true")

    s = sub.add_parser("_supervise")
    s.add_argument("session_dir")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.cmd == "preflight":
        from .preflight import run_preflight, parse_expect
        rep, disc, ctx = run_preflight(args.profile, args.prefix, args.probe_seconds, args.firmware, args.root,
                                       args.min_free_gb, parse_expect(args.expect), image_mode=args.image_mode,
                                       clock_s=args.clock_seconds, camera_api=args.camera_api)
        print(json.dumps(rep.to_dict(), indent=2, default=str) if args.json else rep.render())
        return rep.exit_code
    if args.cmd == "record":
        from .recorder import start
        return start(args)
    if args.cmd == "stop":
        from .recorder import stop
        return stop(args)
    if args.cmd == "inspect":
        from .inspect_cmd import inspect
        return inspect(args)
    if args.cmd == "replay":
        from .replay import replay
        return replay(args)
    if args.cmd == "_supervise":
        from .recorder import supervise
        return supervise(args.session_dir)
    return 2


if __name__ == "__main__":
    sys.exit(main())
