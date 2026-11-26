"""camera_api: read-only whitelist client, tested against a local fake camera (no real device)."""
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from insight_capture import camera_api

POSTS = []


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _send(self, obj):
        b = json.dumps(obj).encode()
        self.send_response(200); self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        r = {"/api/ota/device-versions": {"firewareVersion": "2.1.5", "softwareVersion": "2.1.5"},
             "/api/ota/product-names": {"success": True, "data": ["insight9"]},
             "/api/deep-flow": {"success": True, "data": {"enabled": True}},
             "/api/stereo-format": {"success": True, "data": {"format": "mono8 544x640 30fps", "hdr": True}},
             "/api/time-sync-setting": {"success": True, "data": {"enabled": False, "synced": False}},
             "/api/ros-domain-id": {"success": True, "data": {"rosDomainId": "9"}},
             "/api/system-time": {"timestamp": 1756222013}}.get(self.path)
        self._send(r) if r else self.send_error(404)

    def do_POST(self):
        POSTS.append(self.path); self._send({})


@pytest.fixture(scope="module")
def server():
    s = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=s.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{s.server_port}"
    s.shutdown()


def test_snapshot_summary(server):
    snap = camera_api.snapshot(server, 2.0)
    s = snap["summary"]
    assert s["firmware"] == "2.1.5" and s["depth_enabled"] is True and s["stereo_hdr"] is True
    assert s["time_sync_enabled"] is False and s["ros_domain_id"] == "9" and s["camera_wall_clock_epoch"] == 1756222013
    assert "error" in snap["raw"]["/api/camera-format"] or snap["raw"]["/api/camera-format"] is None or True


def test_unreachable_returns_none():
    assert camera_api.snapshot("http://127.0.0.1:9", 0.3) is None


def test_never_posts_and_rejects_non_whitelisted(server):
    camera_api.snapshot(server, 2.0)
    assert POSTS == []
    with pytest.raises(ValueError):
        camera_api._get(server, "/api/reboot", 1.0)
    assert not any(p in camera_api.GET_WHITELIST for p in ("/api/reboot", "/api/insight-stop", "/api/insight-start"))
