"""Read rosbag2 sqlite3 storage directly (works without metadata.yaml, i.e. after a crash)."""
from __future__ import annotations

import sqlite3
from pathlib import Path

from .stats import analyze, has_header, parse_header_stamp


def db_files(bag: Path) -> list[Path]:
    def idx(p: Path):
        try:
            return int(p.stem.rsplit("_", 1)[1])
        except (IndexError, ValueError):
            return 0
    return sorted(bag.glob("*.db3"), key=idx)


def _connect(p: Path) -> sqlite3.Connection:
    return sqlite3.connect(f"file:{p}?mode=ro", uri=True, timeout=5)


def topics(bag: Path) -> dict[str, dict]:
    """name -> {type, serialization_format, qos} merged over all db files."""
    out: dict[str, dict] = {}
    for f in db_files(bag):
        try:
            con = _connect(f)
            cols = [r[1] for r in con.execute("PRAGMA table_info(topics)")]
            q = "name, type, serialization_format" + (", offered_qos_profiles" if "offered_qos_profiles" in cols else "")
            for r in con.execute(f"SELECT {q} FROM topics"):
                out.setdefault(r[0], {"type": r[1], "serialization_format": r[2],
                                      "qos": r[3] if len(r) > 3 else ""})
            con.close()
        except sqlite3.DatabaseError:
            continue
    return out


def scan(bag: Path, gap_factor: float = 1.5, nominal: dict[str, float] | None = None,
         tf_topics: tuple[str, ...] = ()) -> dict:
    """Per-topic statistics over the whole bag using only the first 12 bytes of each message.

    Returns {"topics": {name: stats}, "files": [...], "errors": [...]}.
    """
    nominal = nominal or {}
    tinfo = topics(bag)
    stamps: dict[str, list] = {n: [] for n in tinfo}
    recv: dict[str, list] = {n: [] for n in tinfo}
    sizes: dict[str, int] = {n: 0 for n in tinfo}
    errors, files = [], []
    for f in db_files(bag):
        files.append({"name": f.name, "size": f.stat().st_size})
        try:
            con = _connect(f)
            names = {r[0]: r[1] for r in con.execute("SELECT id, name FROM topics")}
            cur = con.execute("SELECT topic_id, timestamp, substr(data,1,12), length(data) "
                              "FROM messages ORDER BY id")
            for tid, ts, head, ln in cur:
                n = names.get(tid)
                if n is None:
                    continue
                recv[n].append(ts)
                sizes[n] += ln or 0
                stamps[n].append(parse_header_stamp(head) if has_header(tinfo[n]["type"]) and n not in tf_topics else None)
            con.close()
        except sqlite3.DatabaseError as e:
            errors.append(f"{f.name}: {e}")
    res = {}
    for n in tinfo:
        st = analyze(stamps[n], recv[n], gap_factor, nominal.get(n))
        st["type"] = tinfo[n]["type"]
        st["bytes"] = sizes[n]
        res[n] = st
    return {"topics": res, "files": files, "errors": errors}


def tf_frames(bag: Path, topic_names=("/tf", "/tf_static")) -> set[tuple[str, str]]:
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    edges: set[tuple[str, str]] = set()
    for f in db_files(bag):
        try:
            con = _connect(f)
            ids = [r[0] for r in con.execute("SELECT id FROM topics WHERE name IN (%s)" %
                                             ",".join("?" * len(topic_names)), topic_names)]
            for tid in ids:
                for (data,) in con.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 20000", (tid,)):
                    for t in deserialize_message(data, TFMessage).transforms:
                        edges.add((t.header.frame_id.strip("/"), t.child_frame_id.strip("/")))
            con.close()
        except sqlite3.DatabaseError:
            continue
    return edges


def read_series(bag: Path) -> dict[str, dict]:
    """name -> {'stamps': [...], 'recv': [...], 'type': str} in storage order (header stamps via first 12 bytes)."""
    from .stats import has_header, parse_header_stamp
    tinfo = topics(bag)
    out = {n: {"stamps": [], "recv": [], "type": t["type"]} for n, t in tinfo.items()}
    for f in db_files(bag):
        try:
            con = _connect(f)
            names = {r[0]: r[1] for r in con.execute("SELECT id, name FROM topics")}
            for tid, ts, head in con.execute("SELECT topic_id, timestamp, substr(data,1,12) FROM messages ORDER BY id"):
                n = names.get(tid)
                if n is None:
                    continue
                out[n]["recv"].append(ts)
                out[n]["stamps"].append(parse_header_stamp(head) if has_header(out[n]["type"]) else None)
            con.close()
        except sqlite3.DatabaseError:
            continue
    return out
