"""READ-ONLY access to the Insight 9 web API (the one the vendor web UI uses). Only HTTP GET on a fixed whitelist.

Never POSTs: this tool must not change camera settings. (Experiments that do change settings live in
reports/camera_param_experiment/camera_cfg.py and need explicit user authorisation.)"""
from __future__ import annotations

import json
import urllib.request

DEFAULT_BASE = "http://169.254.10.1"
GET_WHITELIST = ("/api/ota/device-versions", "/api/ota/product-names", "/api/camera-format", "/api/stereo-format",
                 "/api/deep-flow", "/api/time-sync-setting", "/api/sensor-pose-cov", "/api/ros-domain-id",
                 "/api/dds-type", "/api/camera-config", "/api/system-info", "/api/system-time", "/api/mode",
                 "/api/compatibility-mode")


def _get(base: str, path: str, timeout: float):
    if path not in GET_WHITELIST:
        raise ValueError(f"not whitelisted: {path}")
    op = urllib.request.build_opener(urllib.request.ProxyHandler({}))      # shell http_proxy would break link-local access
    with op.open(urllib.request.Request(base + path, method="GET"), timeout=timeout) as f:
        return json.loads(f.read().decode())


def snapshot(base: str = DEFAULT_BASE, timeout: float = 2.0) -> dict | None:
    """Return {'reachable':True,'raw':{path:json},'summary':{...}} or None when the camera API does not answer."""
    raw = {}
    try:
        raw[GET_WHITELIST[0]] = _get(base, GET_WHITELIST[0], timeout)
    except Exception:
        return None
    for p in GET_WHITELIST[1:]:
        try:
            raw[p] = _get(base, p, timeout)
        except Exception as e:
            raw[p] = {"error": str(e)}
    d = lambda p, k: ((raw.get(p) or {}).get("data") or {}).get(k) if isinstance(raw.get(p), dict) else None
    ver = raw[GET_WHITELIST[0]] or {}
    sysinfo = raw.get("/api/system-info") or {}
    tstamp = (raw.get("/api/system-time") or {}).get("timestamp")
    return {"reachable": True, "base": base, "raw": raw, "summary": {
        "firmware": ver.get("firewareVersion"), "software": ver.get("softwareVersion"),
        "product": (raw.get("/api/ota/product-names") or {}).get("data"),
        "color_format": d("/api/camera-format", "format"), "stereo_format": d("/api/stereo-format", "format"),
        "stereo_hdr": d("/api/stereo-format", "hdr"), "depth_enabled": d("/api/deep-flow", "enabled"),
        "time_sync_enabled": d("/api/time-sync-setting", "enabled"), "time_synced": d("/api/time-sync-setting", "synced"),
        "ros_domain_id": d("/api/ros-domain-id", "rosDomainId"), "dds_type": d("/api/dds-type", "ddsType"),
        "camera_wall_clock_epoch": tstamp, "temperature_c": sysinfo.get("temperature"), "uptime": sysinfo.get("uptimeStr")}}
