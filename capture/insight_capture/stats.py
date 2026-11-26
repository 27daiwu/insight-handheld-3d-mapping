"""Timestamp / rate / drop statistics. Pure python (no ROS needed)."""
from __future__ import annotations

import statistics
import struct

NS = 1_000_000_000


def has_header(msg_type: str) -> bool:
    """False for types whose first field is not std_msgs/Header (std_msgs/*, tf2_msgs/TFMessage)."""
    return not (msg_type.startswith("std_msgs/") or msg_type == "tf2_msgs/msg/TFMessage")


def parse_header_stamp(buf: bytes) -> int | None:
    """Header stamp (ns) from the first 12 bytes of a CDR-serialized message
    whose first field is std_msgs/Header. Returns None if unparsable."""
    if buf is None or len(buf) < 12:
        return None
    little = buf[1] & 1  # CDR encapsulation: 0x0001 = CDR_LE, 0x0000 = CDR_BE
    fmt = "<iI" if little else ">iI"
    sec, nsec = struct.unpack_from(fmt, buf, 4)
    return sec * NS + nsec


def parse_frame_id(buf: bytes) -> str | None:
    if buf is None or len(buf) < 16:
        return None
    little = buf[1] & 1
    (n,) = struct.unpack_from("<I" if little else ">I", buf, 12)
    if n == 0 or 16 + n > len(buf):
        return ""
    return buf[16:16 + n - 1].decode("utf-8", "replace")


def analyze(stamps_ns: list[int | None], recv_ns: list[int] | None = None,
            gap_factor: float = 1.5, nominal_hz: float | None = None) -> dict:
    """Statistics for one topic.

    stamps_ns: header stamps in message order (None where the type has no header).
    recv_ns:   receive time (bag timestamp / local clock), same order.
    Drops are inferred from header-stamp gaps larger than gap_factor * median period.
    """
    n = len(stamps_ns)
    out: dict = {"count": n}
    if recv_ns:
        out["recv_first_ns"], out["recv_last_ns"] = recv_ns[0], recv_ns[-1]
        span = (recv_ns[-1] - recv_ns[0]) / NS
        out["recv_span_s"] = round(span, 3)
        out["recv_rate_hz"] = round((n - 1) / span, 3) if n > 1 and span > 0 else None
    valid = [s for s in stamps_ns if s is not None]
    out["stamped"] = len(valid)
    if len(valid) < 2:
        return out
    out["stamp_first_ns"], out["stamp_last_ns"] = valid[0], valid[-1]
    zero = sum(1 for s in valid if s == 0)
    diffs = [b - a for a, b in zip(valid, valid[1:])]
    non_mono = sum(1 for d in diffs if d < 0)
    dup = sum(1 for d in diffs if d == 0)
    pos = [d for d in diffs if d > 0]
    out.update(zero_stamps=zero, non_monotonic=non_mono, duplicate_stamps=dup)
    if not pos:
        return out
    med = statistics.median(pos)
    out["median_period_ms"] = round(med / 1e6, 4)
    out["stamp_rate_hz"] = round(NS / med, 3)
    span = (valid[-1] - valid[0]) / NS
    out["stamp_span_s"] = round(span, 3)
    out["stamp_mean_rate_hz"] = round((len(valid) - 1) / span, 3) if span > 0 else None
    ref = (NS / nominal_hz) if nominal_hz else med
    gaps = [d for d in pos if d > gap_factor * ref]
    missing = sum(max(round(d / ref) - 1, 0) for d in gaps)
    out["gap_events"] = len(gaps)
    out["est_missing_msgs"] = int(missing)
    out["max_gap_ms"] = round(max(pos) / 1e6, 3)
    out["est_drop_ratio"] = round(missing / (len(valid) + missing), 5)
    if len(pos) > 2:
        out["period_jitter_ms"] = round(statistics.pstdev(pos) / 1e6, 4)
    if recv_ns and len(recv_ns) == n:
        offs = [r - s for s, r in zip(stamps_ns, recv_ns) if s is not None]
        out["recv_minus_stamp_median_s"] = round(statistics.median(offs) / NS, 4)
        out["recv_minus_stamp_spread_ms"] = round((max(offs) - min(offs)) / 1e6, 3)
        rd = [b - a for a, b in zip(recv_ns, recv_ns[1:])]
        out["recv_non_monotonic"] = sum(1 for d in rd if d < 0)
    return out


CLOCK_MSG = "camera/host clocks not aligned"


def judge(stat: dict, nominal_hz: float | None, th: dict, clock_check: bool = True) -> list[tuple[str, str]]:
    """Turn stats into (level, message) findings. level in WARN/FAIL.

    clock_check=False omits the per-topic header-vs-receive clock-offset finding (callers report it once, aggregated)."""
    f: list[tuple[str, str]] = []
    if stat.get("count", 0) == 0:
        return [("FAIL", "no messages received")]
    if stat.get("non_monotonic"):
        f.append(("WARN", f"{stat['non_monotonic']} non-monotonic header stamps"))
    if stat.get("zero_stamps"):
        f.append(("WARN", f"{stat['zero_stamps']} zero header stamps"))
    if stat.get("duplicate_stamps"):
        f.append(("WARN", f"{stat['duplicate_stamps']} duplicate header stamps"))
    if stat.get("gap_events"):
        f.append(("WARN", f"{stat['gap_events']} stamp gaps (~{stat['est_missing_msgs']} msgs missing, "
                          f"max gap {stat['max_gap_ms']} ms)"))
    off = stat.get("recv_minus_stamp_median_s")
    if clock_check and off is not None and abs(off) > th.get("max_clock_offset_s", 1.0):
        f.append(("WARN", f"receive time - header stamp = {off:.2f}s ({CLOCK_MSG})"))
    hz = stat.get("stamp_rate_hz") or stat.get("recv_rate_hz")
    if nominal_hz and hz and hz < nominal_hz * th.get("min_rate_ratio", 0.9):
        f.append(("WARN", f"rate {hz:.1f} Hz below reference {nominal_hz:.1f} Hz"))
    return f
