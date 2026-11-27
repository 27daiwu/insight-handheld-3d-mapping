"""Bag reading for MCAP and SQLite3 rosbag2 recordings, with or without a Session wrapper.

* `.mcap` files / MCAP bag directories are read with the pure-python `mcap` + `mcap-ros2-support` libraries
  (no ROS install needed, no metadata.yaml needed, message definitions come from the file itself).
* SQLite3 (`.db3`) bags fall back to rosbag2_py (needs a sourced ROS 2 Humble environment).

Every reader yields raw CDR bytes + the *bag receive time* (`log_ns`) and decodes lazily, so header-only
indexing never deserialises an image.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Optional


class BagOpenError(RuntimeError):
    pass


@dataclass
class Connection:
    topic: str
    msg_type: str
    count: Optional[int]       # None when the container does not carry per-topic counts
    schema_text: str = ""      # message definition if embedded (MCAP)
    qos: str = ""


@dataclass
class Raw:
    topic: str
    msg_type: str
    log_ns: int                # bag receive time (host wall clock at capture, NOT the sensor clock)
    data: bytes                # CDR


def locate_bag(path) -> tuple[Path, str, dict]:
    """Return (bag_dir_or_file, storage, info) for a Session dir, a bag dir or a bare .mcap/.db3 file."""
    p = Path(path)
    info: dict = {"input": str(p)}
    if p.is_file():
        if p.suffix == ".mcap":
            return p, "mcap", info
        if p.suffix == ".db3":
            return p.parent, "sqlite3", info
        raise BagOpenError(f"{p}: not a .mcap/.db3 file")
    if (p / "bag").is_dir():
        info["session_dir"] = str(p)
        p = p / "bag"
    if not p.is_dir():
        raise BagOpenError(f"{p}: no such bag")
    mcaps = sorted(p.glob("*.mcap"))
    dbs = sorted(p.glob("*.db3"))
    if mcaps:
        return p, "mcap", info
    if dbs:
        return p, "sqlite3", info
    raise BagOpenError(f"{p}: contains neither .mcap nor .db3 files")


class BagSource:
    def __init__(self, path):
        self.path, self.storage, self.info = locate_bag(path)
        self.files = ([self.path] if self.path.is_file() else sorted(self.path.glob("*.mcap"))) if self.storage == "mcap" else []
        self._decoders: dict = {}
        self._ros = None
        if self.storage == "sqlite3":
            from ..bagio import BagReader
            self._ros = BagReader(self.path)
        self._conns: Optional[list] = None

    # ------------------------------------------------------------------ metadata
    def connections(self) -> list[Connection]:
        if self._conns is not None:
            return self._conns
        out: dict[str, Connection] = {}
        if self.storage == "mcap":
            from mcap.reader import make_reader
            for f in self.files:
                with open(f, "rb") as fh:
                    r = make_reader(fh)
                    s = r.get_summary()
                    if s is None:
                        raise BagOpenError(f"{f}: no MCAP summary section (recording not closed?); reindex with `mcap recover`")
                    for cid, ch in s.channels.items():
                        sch = s.schemas.get(ch.schema_id)
                        n = s.statistics.channel_message_counts.get(cid) if s.statistics else None
                        c = out.get(ch.topic)
                        if c is None:
                            out[ch.topic] = Connection(ch.topic, sch.name if sch else "?", n,
                                                       sch.data.decode("utf-8", "replace") if sch else "",
                                                       ch.metadata.get("offered_qos_profiles", ""))
                        elif n is not None:
                            c.count = (c.count or 0) + n
        else:
            counts = self._ros.message_counts()
            for t, ty in self._ros.topics().items():
                out[t] = Connection(t, ty, counts.get(t))
        self._conns = sorted(out.values(), key=lambda c: c.topic)
        return self._conns

    def type_of(self, topic: str) -> str:
        return next(c.msg_type for c in self.connections() if c.topic == topic)

    # ------------------------------------------------------------------ iteration
    def iter_raw(self, topics: Optional[Iterable[str]] = None) -> Iterator[Raw]:
        topics = list(topics) if topics is not None else None
        if self.storage == "mcap":
            from mcap.reader import make_reader
            for f in self.files:
                with open(f, "rb") as fh:
                    r = make_reader(fh)
                    for sch, ch, m in r.iter_messages(topics=topics, log_time_order=True):
                        self._decoders.setdefault((ch.id, f.name), (sch, ch))
                        yield Raw(ch.topic, sch.name if sch else "?", int(m.log_time), m.data)
        else:
            for raw in self._ros.iter_raw(topics):
                yield Raw(raw.topic, raw.msg_type, raw.bag_ns, raw.data)

    def decode(self, raw: Raw):
        if self.storage == "sqlite3":
            from ..bagio import RawMsg
            return self._ros.deserialize(RawMsg(raw.topic, raw.log_ns, raw.data, raw.msg_type))
        key = raw.msg_type
        dec = self._decoders.get(("fn", key))
        if dec is None:
            from mcap.records import Schema
            from mcap_ros2.decoder import DecoderFactory
            sch_text = next((c.schema_text for c in self.connections() if c.msg_type == key), None)
            if sch_text is None:
                raise BagOpenError(f"no schema for {key}")
            sch = Schema(id=1, name=key, encoding="ros2msg", data=sch_text.encode())
            dec = DecoderFactory().decoder_for("cdr", sch)
            self._decoders[("fn", key)] = dec
        return dec(raw.data)

    def iter_msgs(self, topics: Optional[Iterable[str]] = None):
        for raw in self.iter_raw(topics):
            yield raw, self.decode(raw)
