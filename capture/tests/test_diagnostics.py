"""Continuity / loss / systemic-hole / sync diagnostics on SYNTHETIC data (never evidence about a real camera)."""
import json
import types

from conftest import NS, make_bag, series
from insight_capture import diagnostics as D
from insight_capture.inspect_cmd import finalize_manifest, inspect
from insight_capture.session import write_json

TH = {"max_loss_ratio": 0.001}


def ser(hz, n, holes=(), t0=1000 * NS, name_off=0):
    """series with time windows [a,b) seconds (relative) removed."""
    p = NS / hz
    pts = [(int(t0 + i * p + name_off), int(t0 + i * p + name_off)) for i in range(n)]
    pts = [x for x in pts if not any(t0 + a * NS <= x[0] < t0 + b * NS for a, b in holes)]
    return {"stamps": [x[1] for x in pts], "recv": [x[0] for x in pts]}


def test_gaps_and_missing_count():
    s = ser(30, 300, holes=[(2.0, 2.5)])
    g = D.gaps(s["stamps"])
    assert len(g) == 1 and g[0]["missing"] in (14, 15) and 500 <= g[0]["dur_ms"] <= 540


def test_loss_vs_reference_rate_catches_uniformly_slow_stream():
    # stream runs at 20 Hz but 30 Hz was configured: no gaps vs own median, 33% loss vs reference
    s = ser(20, 200)
    c = D.continuity(s["stamps"], s["recv"], nominal_hz=30)
    assert c["loss_vs_median"] == 0 and 0.30 < c["loss_vs_reference"] < 0.36


def test_clean_series_has_no_findings():
    series_ = {"/a": ser(30, 600), "/b": ser(100, 2000)}
    rep = D.diagnose(series_, {"/a": 30, "/b": 100})
    assert D.verdicts(rep, TH) == []
    assert rep["systemic"]["systemic_events"] == 0


def test_single_topic_gap_is_not_systemic():
    series_ = {"/a": ser(30, 900, holes=[(5, 7)]), "/b": ser(100, 3000), "/c": ser(30, 900), "/d": ser(30, 900)}
    rep = D.diagnose(series_)
    assert rep["systemic"]["systemic_events"] == 0
    assert any("/a" in m for _, m in D.verdicts(rep, TH))


def test_periodic_multitopic_hole_is_detected_like_the_wsl2_case():
    holes = [(6.9 + 34.5 * k, 9.3 + 34.5 * k) for k in range(8)]       # ~2.4 s every 34.5 s
    names = {"/rgb": 30, "/left": 30, "/right": 30, "/imu": 100, "/vio": 30}
    series_ = {n: ser(hz, int(hz * 300), holes=holes) for n, hz in names.items()}
    rep = D.diagnose(series_, names)
    s = rep["systemic"]
    assert s["systemic_events"] == 8 and s["periodic"] is True
    assert abs(s["period_s"]["median"] - 34.5) < 0.3
    v = D.verdicts(rep, TH)
    assert any(l == "FAIL" and "multi-topic holes" in m and "periodic" in m for l, m in v)


def test_pair_skew_and_unmatched():
    a = ser(30, 300)["stamps"]
    b = [x + 3_000_000 for x in a]                                      # 3 ms skew
    r = D.pair_skew(a, b)
    assert r["match_ratio"] == 1.0 and abs(r["mean_ms"] - 3) < 0.01
    b2 = [x for i, x in enumerate(b) if i % 4 != 0]                    # right camera lost 25 %
    assert D.pair_skew(a, b2)["match_ratio"] < 0.8 + 0.05
    rep = D.diagnose({"/l": {"stamps": a, "recv": a}, "/r": {"stamps": b2, "recv": b2}}, None, [("/l", "/r")])
    assert any("sync" in m for _, m in D.verdicts(rep, TH))


def test_non_monotonic_reported():
    s = ser(30, 100)
    s["stamps"][50], s["stamps"][51] = s["stamps"][51], s["stamps"][50]
    c = D.continuity(s["stamps"], s["recv"])
    assert c["non_monotonic"] == 1


def test_stamp_gap_with_continuous_receive_is_distinguishable():
    """camera stamped a hole but host kept receiving (e.g. burst/late) -> recv gap count 0, stamp gap 1."""
    s = ser(30, 300)
    stamps = list(s["stamps"])
    for i in range(100, 300):
        stamps[i] += 2 * NS                                             # stamp clock jumps +2 s, arrival unchanged
    rep = D.diagnose({"/x": {"stamps": stamps, "recv": s["recv"]}})
    t = rep["topics"]["/x"]
    assert t["gap_events"] == 1 and t["recv_gap_events"] == 0


def test_inspect_flags_systemic_holes_in_a_bag(tmp_path, capsys):
    holes = [(6.9 + 34.5 * k, 9.3 + 34.5 * k) for k in range(4)]
    topics, plan = {}, []
    for role, name, typ, hz in [("rgb", "/c/rgb/image_raw", "sensor_msgs/msg/Image", 30),
                                ("stereo_left", "/c/left/image_raw", "sensor_msgs/msg/Image", 30),
                                ("imu", "/c/imu", "sensor_msgs/msg/Imu", 100),
                                ("vio", "/c/vio", "nav_msgs/msg/Odometry", 30)]:
        s = ser(hz, hz * 150, holes=holes)
        topics[name] = (typ, list(zip(s["recv"], s["stamps"])))
        plan.append({"role": role, "name": name, "type": typ, "required": True, "reference_hz": float(hz)})
    sd = tmp_path / "s1"
    make_bag(sd / "bag", topics)
    write_json(sd / "manifest.json", {"schema_version": "x", "session_id": "s1", "status": "recording",
                                      "camera": {"firmware": "x"}, "time": {}, "topics": plan, "bag": {},
                                      "depth": {"enabled": False}, "host": {}, "disk": {}})
    write_json(sd / "session.json", {"status": "complete", "supervisor_pid": None})
    finalize_manifest(sd, "complete", "user_stop", 0, [])
    rc = inspect(types.SimpleNamespace(root=str(tmp_path), session="s1", profile=None, json=True, repair=False,
                                       no_tf=True))
    out = json.loads(capsys.readouterr().out)
    diag = [c for c in out["checks"] if c["id"] == "diag"]
    assert any(c["level"] == "FAIL" and "multi-topic" in c["title"] for c in diag) and rc == 2
    assert (sd / "diagnostics.json").exists()
    assert out["diagnostics"]["systemic"]["systemic_events"] == 4


def test_small_jitter_gaps_do_not_chain_into_a_fake_long_hole():
    """regression: real-camera run produced a bogus 309 s 'systemic hole' from chained ms-scale gaps."""
    series_ = {}
    for n in ("/a", "/b", "/c", "/d"):
        s = ser(10, 3000)
        keep = [i for i in range(len(s["recv"])) if i % 7 != 0]      # drop every 7th msg -> 200 ms gaps everywhere
        series_[n] = {"stamps": [s["stamps"][i] for i in keep], "recv": [s["recv"][i] for i in keep]}
    rep = D.diagnose(series_)
    assert rep["systemic"]["systemic_events"] == 0


def clock_step_series(hz, n, steps_at, step_s=2.4, t0=1000 * NS):
    """Header stamps contiguous (camera clock); receive time = host wall clock that steps forward then slews back,
    i.e. what the WSL2 timesyncd sawtooth does. No message is lost."""
    p = NS / hz
    stamps = [int(5 * NS + i * p) for i in range(n)]
    recv = []
    for i in range(n):
        t = i * p / NS
        off = 0.0
        for s0 in steps_at:
            if t >= s0:
                off = step_s * max(0.0, 1 - (t - s0) / 20.0) if t - s0 < 20 else off      # 0..20 s slew back
        # forward step at s0 then ramps back to 0 over 20 s
        recv.append(int(t0 + i * p + off * NS))
    return {"stamps": stamps, "recv": recv}


def test_host_clock_steps_are_flagged_as_recv_only_not_data_loss():
    steps = [10, 42, 74, 106]
    series_ = {n: clock_step_series(hz, hz * 130, steps) for n, hz in
               {"/rgb": 30, "/left": 30, "/imu": 100, "/vio": 30}.items()}
    rep = D.diagnose(series_, {"/rgb": 30, "/left": 30, "/imu": 100, "/vio": 30})
    s = rep["systemic"]
    assert s["systemic_events"] >= 3 and s["data_loss_events"] == 0 and s["recv_only_events"] == s["systemic_events"]
    assert all(t["gap_events"] == 0 for t in rep["topics"].values())          # header stamps are contiguous
    v = D.verdicts(rep, TH)
    assert not any(l == "FAIL" for l, _ in v) and any("NOT lost" in m for _, m in v)


def test_real_loss_in_both_stamp_and_recv_is_still_fail():
    holes = [(6.9 + 34.5 * k, 9.3 + 34.5 * k) for k in range(4)]
    series_ = {n: ser(hz, hz * 150, holes=holes) for n, hz in {"/a": 30, "/b": 30, "/c": 100}.items()}
    rep = D.diagnose(series_, {"/a": 30, "/b": 30, "/c": 100})
    assert rep["systemic"]["data_loss_events"] == 4
    assert any(l == "FAIL" for l, _ in D.verdicts(rep, TH))


def test_unstamped_topic_does_not_block_recv_only_classification():
    steps = [10, 42, 74]
    series_ = {n: clock_step_series(hz, hz * 100, steps) for n, hz in {"/rgb": 30, "/imu": 100, "/vio": 30}.items()}
    tf = clock_step_series(50, 5000, steps)
    series_["/tf"] = {"stamps": [None] * len(tf["recv"]), "recv": tf["recv"]}        # TFMessage has no header
    rep = D.diagnose(series_)
    assert rep["systemic"]["data_loss_events"] == 0 and rep["systemic"]["recv_only_events"] >= 2
    assert not any("/tf" in m for _, m in D.verdicts(rep, TH))
