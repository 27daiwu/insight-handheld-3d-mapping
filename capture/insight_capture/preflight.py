"""Pre-capture checks. Read-only: subscribes and inspects, never changes camera/system config."""
from __future__ import annotations

import json
import os
from collections import defaultdict, deque

from . import sysinfo
from .config import load_profile
from .discovery import discover, Discovery
from .rosgraph import Graph
from .stats import judge

LEVELS = {"PASS": 0, "INFO": 0, "WARN": 1, "FAIL": 2}


class Report:
    def __init__(self):
        self.checks: list[dict] = []
        self.data: dict = {}

    def add(self, level: str, cid: str, title: str, detail: str = ""):
        self.checks.append({"level": level, "id": cid, "title": title, "detail": detail})

    @property
    def exit_code(self) -> int:
        return max([LEVELS[c["level"]] for c in self.checks] or [0])

    def to_dict(self) -> dict:
        return {"exit_code": self.exit_code, "checks": self.checks, **self.data}

    def render(self) -> str:
        sym = {"PASS": "[ OK ]", "INFO": "[INFO]", "WARN": "[WARN]", "FAIL": "[FAIL]"}
        lines = []
        for c in self.checks:
            lines.append(f"{sym[c['level']]} {c['title']}")
            for dl in filter(None, c["detail"].split("\n")):
                lines.append(f"         {dl}")
        n = {k: sum(1 for c in self.checks if c["level"] == k) for k in LEVELS}
        lines.append(f"\nsummary: {n['PASS']} ok, {n['WARN']} warn, {n['FAIL']} fail  (exit {self.exit_code})")
        return "\n".join(lines)


def _connected(edges: set[tuple[str, str]], frames: list[str]) -> tuple[bool, list[str]]:
    adj = defaultdict(set)
    for a, b in edges:
        adj[a].add(b); adj[b].add(a)
    frames = [f.strip("/") for f in frames if f]
    if len(set(frames)) < 2:
        return True, []
    seen = {frames[0]}
    q = deque([frames[0]])
    while q:
        for n in adj[q.popleft()]:
            if n not in seen:
                seen.add(n); q.append(n)
    miss = sorted({f for f in frames if f not in seen})
    return not miss, miss


def parse_expect(items: list[str] | None) -> dict[str, float]:
    out = {}
    for it in items or []:
        k, v = it.split("=", 1)
        out[k.strip()] = float(v)
    return out


def run_preflight(profile_path=None, prefix=None, probe_s: float = 5.0, firmware: str | None = None,
                  data_root=None, min_free_gb: float | None = None, expect: dict | None = None,
                  settle_s: float = 2.0, image_mode: str = "raw", quiet_system: bool = False,
                  clock_s: float = 0.0
                  ) -> tuple[Report, Discovery | None, dict]:
    prof = load_profile(profile_path)
    th = prof["thresholds"]
    min_free = min_free_gb if min_free_gb is not None else th["min_free_gb"]
    expect = expect or {}
    rep = Report()
    ctx: dict = {"probe": {}, "topic_infos": []}

    # 1. OS / ROS --------------------------------------------------------
    osi, ros = sysinfo.os_info(), sysinfo.ros_info()
    rep.data["system"] = {"os": osi, "ros": ros}
    rep.add("PASS" if "22.04" in osi.get("os", "") else "WARN", "os",
            f"OS: {osi.get('os')} / kernel {osi['kernel']}",
            "" if "22.04" in osi.get("os", "") else "Humble is only fully supported on Ubuntu 22.04")
    rep.add("PASS" if ros["distro"] == "humble" else "FAIL", "ros_distro", f"ROS_DISTRO={ros['distro']}",
            "" if ros["distro"] == "humble" else "source /opt/ros/humble/setup.bash")
    fast = ros["rmw_env"] in (None, "", "rmw_fastrtps_cpp", "rmw_fastrtps_dynamic_cpp")
    rep.add("PASS" if fast else "WARN", "rmw", f"RMW: {ros['rmw_effective']}",
            "" if fast else "Fast DDS is the project baseline; RMW_IMPLEMENTATION points elsewhere")
    rep.add("INFO", "ros_net", f"ROS_DOMAIN_ID={ros['domain_id']} ROS_LOCALHOST_ONLY={ros['localhost_only']}",
            f"Fast DDS profile: {ros['fastdds_profile'] or 'none'}")
    bag = sysinfo.rosbag2_info()
    ok = bag["ok"] and "sqlite3" in bag["storage_plugins"]
    rep.add("PASS" if ok else "FAIL", "rosbag2", "rosbag2 with sqlite3 storage plugin",
            "" if ok else (bag["error"] or f"plugins: {bag['storage_plugins']}"))

    # 2. firmware --------------------------------------------------------
    fw = firmware or os.environ.get("INSIGHT_FIRMWARE")
    base = prof.get("firmware_baseline")
    ctx["firmware"] = fw
    if not fw:
        rep.add("WARN", "firmware", "Firmware version: unknown",
                f"No ROS interface for firmware version is known; pass --firmware (baseline {base}). "
                "It is recorded in manifest.json.")
    elif fw.lstrip("v") != base:
        rep.add("WARN", "firmware", f"Firmware {fw} differs from baseline {base}", "topic profile may not match")
    else:
        rep.add("PASS", "firmware", f"Firmware {fw} (baseline {base})")

    # 3. USB / network ---------------------------------------------------
    usb, net = sysinfo.usb_info(), sysinfo.net_info()
    rep.data["system"].update(usb=usb, net=net)
    if usb["lsusb"]:
        usb_detail = f"lsusb unavailable: {usb['lsusb']}"
    else:
        usb_detail = f"{len(usb['devices'])} USB devices; video nodes: {usb['video_nodes'] or 'none'}"
        hit = [d for d in usb["devices"] if "insight" in d.lower()]
        if hit:
            usb_detail += "\nInsight-like device: " + "; ".join(hit)
    if osi["wsl"] and not usb["usb_bus_present"]:
        rep.add("WARN", "usb", "USB: no /dev/bus/usb (WSL2 without USB passthrough)",
                "A USB-attached camera is invisible here. Attach via usbipd-win (needs your OK; not done "
                "automatically) or run capture on native Ubuntu.\n" + usb_detail)
    else:
        rep.add("INFO", "usb", "USB bus", usb_detail)
    ifs = [f"{i['name']} {i['state']} {' '.join(a for a in i['addrs'] if '.' in a)}" for i in net["interfaces"]]
    ll = [i["name"] for i in net["interfaces"] if any(a.startswith("169.254.") for a in i["addrs"])]
    rep.add("INFO", "network", "Network interfaces", "\n".join(ifs) +
            (f"\nlink-local (169.254/16) on: {', '.join(ll)} - possible direct-attached device; unverified"
             if ll else ""))

    # 4. disk ------------------------------------------------------------
    from .config import data_root as _dr
    root = _dr(data_root)
    disk = sysinfo.disk_info(root)
    ctx["disk"] = disk
    rep.add("FAIL" if disk["free_gb"] < min_free else "PASS", "disk",
            f"Disk free {disk['free_gb']} GB at {root}", f"minimum required {min_free} GB")

    # 5. ROS graph / discovery ------------------------------------------
    dok, dmsg = sysinfo.ensure_ros2_daemon()
    rep.add("PASS" if dok else "WARN", "ros2_daemon", "ros2 daemon running" if dok else "ros2 daemon could not be started",
            "" if dok else dmsg + "\nDDS discovery between processes may be unreliable on this host.")
    try:
        g = Graph().__enter__()
    except Exception as e:  # rclpy not importable / context failure
        rep.add("FAIL", "rclpy", "Cannot start rclpy", str(e))
        return rep, None, ctx
    try:
        infos = g.snapshot(settle_s)
        ctx["topic_infos"] = infos
        disc = discover(infos, prof, prefix)
        for _ in range(4):   # DDS discovery can lag on first start (seen on WSL2 right after the daemon starts): re-poll before declaring absence
            if disc.found:
                break
            infos = g.snapshot(3.0)
            disc = discover(infos, prof, prefix)
        ctx["topic_infos"] = infos
        rep.data["discovery"] = disc.to_dict()
        nodes = g.node_names()
        if not disc.found:
            foreign = sorted({t.name for t in infos if t.name not in ("/rosout", "/parameter_events")})
            rep.add("FAIL", "camera_found", "No Insight camera topics discovered",
                    f"{len(infos)} topics / {len(nodes)} nodes visible in this domain, none match the "
                    f"profile '{prof['profile']}' (verified={prof.get('verified')}).\n"
                    "Is the camera powered/attached and its ROS driver running? Check ROS_DOMAIN_ID, "
                    "USB/network, and `ros2 topic list -t`. Not rewriting the SDK driver (per task scope).\n"
                    f"Visible (first 8): {foreign[:8]}")
            return rep, disc, ctx
        rep.add("PASS", "camera_found", f"Camera prefix: '{disc.prefix or '/'}'",
                "\n".join(disc.notes) or f"profile {prof['profile']} (verified={prof.get('verified')})")
        if not prof.get("verified"):
            rep.add("WARN", "profile_unverified", "Topic profile is not verified against v2.1.4",
                    "edit config/topics_v2.1.4.yaml once the real topic list is known")

        sel = disc.chosen(image_mode)
        for role in disc.missing_required:
            rep.add("FAIL", f"missing_{role}", f"Required role '{role}' not found")
        for role, ms in sel.items():
            if role == "depth":
                continue
            for m in ms:
                rep.add("PASS" if m.pub_count else "FAIL", f"topic_{role}",
                        f"{role}: {m.topic} [{m.type}] pubs={m.pub_count}")
        _depth_check(rep, disc, sel)

        # 6a. host wall clock (runs concurrently with the probe) -----------
        import threading
        clk: dict = {}
        cth = None
        if clock_s > 0:
            cth = threading.Thread(target=lambda: clk.update(sysinfo.clock_steps(clock_s)), daemon=True)
            cth.start()
        # 6. probe -------------------------------------------------------
        targets = {m.topic: m.type for ms in sel.values() for m in ms if m.pub_count or m.role == "tf_static"}
        if disc.found and probe_s > 0 and targets:
            probe = g.probe(targets, probe_s, th["gap_factor"], None)
            ctx["probe"] = probe
            rep.data["probe"] = {k: {kk: vv for kk, vv in v.items() if kk != "edges"} for k, v in probe.items()}
            _judge_probe(rep, sel, probe, th, expect)
            _info_tf_checks(rep, sel, probe)
            _disk_estimate(rep, probe, disk, min_free)
        if cth is not None:
            cth.join(clock_s + 5)
            ts = sysinfo.timesync_info()
            rep.data["host_clock"] = {**clk, **ts}
            if clk.get("n_steps"):
                rep.add("WARN", "host_clock",
                        f"Host wall clock STEPPED {clk['n_steps']}x in {clock_s:.0f}s: {clk['steps'][:6]} (t_s, ms)",
                        "rosbag2 stamps with the wall clock -> fake multi-topic holes/bursts in receive time. "
                        "Header stamps are unaffected. timesyncd active: %s. Known on this WSL2 host: timesyncd "
                        "32 s polls step the clock ~+2.4 s (see reports/2_wsl2_packet_loss_rca.md). No change made." %
                        ts.get("timesyncd_active"))
            else:
                rep.add("PASS", "host_clock", f"Host wall clock steady (no step > 50 ms in {clock_s:.0f}s)")
        nodes_note = ", ".join(nodes[:10])
        rep.add("INFO", "nodes", f"{len(nodes)} ROS nodes visible", nodes_note)
    finally:
        g.__exit__(None, None, None)
    return rep, disc, ctx


def _depth_check(rep: Report, disc: Discovery, sel: dict):
    if not disc.depth_topic_present:
        rep.add("WARN", "depth", "Depth NOT enabled (no depth topic)",
                "Depth is off by default. It will not be recorded. This tool never changes camera "
                "settings; enable it in the camera's own config if needed.")
    elif not disc.depth_publishing:
        rep.add("WARN", "depth", "Depth topic exists but has no publisher - Depth NOT enabled",
                "Will not be recorded; no camera setting was changed.")
    else:
        for m in sel.get("depth", []):
            rep.add("PASS", "depth", f"Depth enabled: {m.topic}")


def _judge_probe(rep, sel, probe, th, expect):
    for role, ms in sel.items():
        for m in ms:
            st = probe.get(m.topic)
            if st is None:
                continue
            hz = st.get("stamp_rate_hz") or st.get("recv_rate_hz")
            if st["count"] == 0:
                lvl = "WARN" if role in ("tf_static", "depth", "depth_info") else "FAIL"
                rep.add(lvl, f"rate_{role}", f"{role}: no messages in probe window", m.topic)
                continue
            findings = judge(st, expect.get(role), th)
            summ = (f"{role}: {st['count']} msgs, {hz if hz is not None else 'n/a'} Hz"
                    f" (recv {st.get('recv_rate_hz')}), drops~{st.get('est_missing_msgs', 0)},"
                    f" {st['bytes_per_s'] / 1e6:.2f} MB/s")
            if not findings:
                rep.add("PASS", f"rate_{role}", summ)
            else:
                worst = "FAIL" if any(l == "FAIL" for l, _ in findings) else "WARN"
                rep.add(worst, f"rate_{role}", summ, "\n".join(msg for _, msg in findings))


def _info_tf_checks(rep, sel, probe):
    # CameraInfo
    for role, ms in sel.items():
        spec_for = role.endswith("_info")
        for m in ms if spec_for else []:
            f = (probe.get(m.topic) or {}).get("first") or {}
            if not f:
                continue
            base = role[:-5]
            img = next((probe.get(x.topic, {}).get("first") or {} for x in sel.get(base, [])), {})
            probs = []
            if not f.get("calibrated"):
                probs.append("K matrix all zeros (uncalibrated)")
            if img.get("width") and f.get("width") and (img["width"], img["height"]) != (f["width"], f["height"]):
                probs.append(f"size {f['width']}x{f['height']} != image {img['width']}x{img['height']}")
            if img.get("frame_id") and f.get("frame_id") and img["frame_id"] != f["frame_id"]:
                probs.append(f"frame_id '{f['frame_id']}' != image '{img['frame_id']}'")
            rep.add("WARN" if probs else "PASS", f"caminfo_{role}",
                    f"CameraInfo {role}: {f.get('width')}x{f.get('height')} frame={f.get('frame_id')}",
                    "; ".join(probs))
    # TF
    edges = set()
    for k in ("tf", "tf_static"):
        for m in sel.get(k, []):
            edges.update(map(tuple, (probe.get(m.topic) or {}).get("edges", [])))
    frames = []
    for role, ms in sel.items():
        if role in ("tf", "tf_static") or role.endswith("_info"):
            continue
        for m in ms:
            fid = ((probe.get(m.topic) or {}).get("first") or {}).get("frame_id")
            if fid:
                frames.append(fid)
    if not edges:
        rep.add("WARN", "tf", "No TF transforms observed in probe window")
    else:
        ok, miss = _connected(edges, frames)
        rep.add("PASS" if ok else "WARN", "tf", f"TF: {len(edges)} edges, data frames {sorted(set(frames))}",
                "" if ok else f"frames not connected to the rest of the tree: {miss}")


def _disk_estimate(rep, probe, disk, min_free):
    bps = sum(v.get("bytes_per_s", 0) for v in probe.values())
    if not bps:
        return
    gb5 = bps * 300 / 1e9
    hr = bps * 3600 / 1e9
    free = disk["free_gb"] - min_free
    lvl = "FAIL" if free < gb5 else ("WARN" if free < hr else "PASS")
    rep.add(lvl, "disk_budget", f"Data rate {bps / 1e6:.1f} MB/s: 5 min = {gb5:.1f} GB, 1 h = {hr:.0f} GB",
            f"usable free space (above {min_free} GB reserve): {free:.0f} GB")
