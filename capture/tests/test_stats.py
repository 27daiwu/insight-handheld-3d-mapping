from conftest import NS, cdr_header, series
from insight_capture.stats import analyze, judge, parse_frame_id, parse_header_stamp

TH = {"gap_factor": 1.5, "min_rate_ratio": 0.9, "max_clock_offset_s": 1.0}


def test_header_parse_roundtrip():
    b = cdr_header(1234 * NS + 567, "camera_link")
    assert parse_header_stamp(b[:12]) == 1234 * NS + 567
    assert parse_frame_id(b) == "camera_link"
    assert parse_header_stamp(b"") is None


def test_clean_series():
    s = series(30, 300)
    st = analyze([x[1] for x in s], [x[0] for x in s])
    assert st["count"] == 300 and abs(st["stamp_rate_hz"] - 30) < 0.1
    assert st["est_missing_msgs"] == 0 and judge(st, 30, TH) == []


def test_detects_drops():
    s = series(30, 300, skip={50, 51, 120})
    st = analyze([x[1] for x in s], [x[0] for x in s])
    assert st["gap_events"] == 2 and st["est_missing_msgs"] == 3
    assert any("gaps" in m for _, m in judge(st, 30, TH))


def test_non_monotonic_and_duplicates():
    stamps = [1 * NS, 2 * NS, 2 * NS, int(1.5 * NS), 3 * NS]
    st = analyze(stamps, [i * NS for i in range(5)])
    assert st["non_monotonic"] == 1 and st["duplicate_stamps"] == 1
    assert len(judge(st, None, TH)) >= 2


def test_clock_offset_flagged():
    s = series(30, 100, offset_ns=5 * NS)
    st = analyze([x[1] for x in s], [x[0] for x in s])
    assert abs(st["recv_minus_stamp_median_s"] - 5) < 0.01
    assert any("clock" in m or "header stamp" in m for _, m in judge(st, 30, TH))


def test_low_rate_vs_reference():
    s = series(20, 100)
    st = analyze([x[1] for x in s], [x[0] for x in s])
    assert any("below reference" in m for _, m in judge(st, 30, TH))


def test_empty_and_unstamped():
    assert judge(analyze([], []), 30, TH)[0][0] == "FAIL"
    st = analyze([None] * 5, [i * NS for i in range(5)])
    assert st["stamped"] == 0 and st["recv_rate_hz"] == 1.0
