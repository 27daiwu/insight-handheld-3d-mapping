import sqlite3
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

NS = 10**9


def cdr_header(stamp_ns: int, frame_id: str = "cam", pad: int = 8) -> bytes:
    fid = frame_id.encode() + b"\0"
    return (b"\x00\x01\x00\x00" + struct.pack("<iI", stamp_ns // NS, stamp_ns % NS)
            + struct.pack("<I", len(fid)) + fid + b"\0" * pad)


def make_bag(bag: Path, topics: dict) -> Path:
    """topics: name -> (type, [(recv_ns, stamp_ns or None), ...]) ; writes bag_0.db3 (rosbag2 sqlite3 schema)."""
    bag.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(bag / "bag_0.db3")
    con.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT, "
                "offered_qos_profiles TEXT)")
    con.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
    rows = []
    for tid, (name, (typ, msgs)) in enumerate(topics.items(), 1):
        con.execute("INSERT INTO topics VALUES(?,?,?,?,?)", (tid, name, typ, "cdr", ""))
        for recv, stamp in msgs:
            rows.append((tid, recv, cdr_header(stamp) if stamp is not None else b"\x00\x01\x00\x00" + b"\0" * 16))
    rows.sort(key=lambda r: r[1])
    con.executemany("INSERT INTO messages(topic_id, timestamp, data) VALUES(?,?,?)", rows)
    con.commit(); con.close()
    return bag


def series(hz: float, n: int, t0: int = 1_000 * NS, skip=(), offset_ns: int = 0):
    p = int(NS / hz)
    return [(t0 + i * p + offset_ns, t0 + i * p) for i in range(n) if i not in skip]


@pytest.fixture
def profile():
    from insight_capture.config import load_profile
    return load_profile()
