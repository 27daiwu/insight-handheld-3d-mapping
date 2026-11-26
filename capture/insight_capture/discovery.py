"""Prefix-agnostic topic discovery. Pure functions: no ROS import needed."""
from __future__ import annotations

import re
from dataclasses import dataclass, field, asdict


@dataclass
class TopicInfo:
    name: str
    type: str
    pub_count: int = 0
    qos: str = ""          # human readable, informational only


@dataclass
class Match:
    role: str
    topic: str
    type: str
    pub_count: int = 0
    compressed: bool = False


@dataclass
class Discovery:
    prefix: str | None = None
    found: bool = False
    candidates: dict = field(default_factory=dict)        # prefix -> score
    matches: dict = field(default_factory=dict)          # role -> [Match] (preferred first)
    missing_required: list = field(default_factory=list)
    missing_optional: list = field(default_factory=list)
    extras: list = field(default_factory=list)
    depth_topic_present: bool = False
    depth_publishing: bool = False
    notes: list = field(default_factory=list)

    def chosen(self, image_mode: str = "raw") -> dict:
        """role -> [Match] actually selected for recording."""
        out = {}
        for role, ms in self.matches.items():
            if not ms:
                continue
            raw = [m for m in ms if not m.compressed]
            comp = [m for m in ms if m.compressed]
            if image_mode == "both":
                out[role] = ms
            elif image_mode == "compressed":
                out[role] = comp or raw[:1]
            else:
                out[role] = raw[:1] or comp[:1]
        return out

    def to_dict(self) -> dict:
        d = asdict(self)
        return d


def _is_compressed(name: str) -> bool:
    return name.endswith("/compressed") or name.endswith("/compressedDepth")


def _rank(m) -> tuple:
    """raw before compressed; *_rect_raw before *_raw (on fw 2.1.5 only the rectified topics carry data)."""
    return (m.compressed, 0 if "rect" in m.topic else 1, m.topic)


def _norm(name: str) -> str:
    return "/" + name.strip("/")


def discover(topics: list[TopicInfo], profile: dict, prefix: str | None = None) -> Discovery:
    """Match roles to topics. If `prefix` is None choose the best-scoring prefix.

    Score = number of *core* roles (required roles) matched under the prefix.
    A prefix is accepted when it covers IMU or at least 2 required roles, which
    avoids latching onto an unrelated device sharing the DDS domain.
    """
    roles = profile["roles"]
    compiled = {r: re.compile("^(?P<prefix>.*?)(?:" + spec["suffix"] + ")$") for r, spec in roles.items()}

    per_prefix: dict[str, dict[str, list[Match]]] = {}
    for t in topics:
        name = _norm(t.name)
        for role, spec in roles.items():
            if t.type not in spec["types"]:
                continue
            m = compiled[role].match(name)
            if not m:
                continue
            pfx = m.group("prefix")
            per_prefix.setdefault(pfx, {}).setdefault(role, []).append(
                Match(role, name, t.type, t.pub_count, _is_compressed(name)))

    required = [r for r, s in roles.items() if s.get("required")]
    scores = {p: sum(1 for r in required if r in rm) for p, rm in per_prefix.items()}

    d = Discovery(candidates=scores)
    if prefix is not None:
        chosen = "" if prefix in ("", "/") else _norm(prefix)
        d.prefix = chosen
        rm = per_prefix.get(chosen, {})
        d.found = bool(rm)
    else:
        ok = {p: s for p, s in scores.items() if s >= 2 or "imu" in per_prefix[p]}
        if ok:
            # best score, then prefer publishers present, then shortest prefix
            def key(p):
                pubs = sum(m.pub_count for ms in per_prefix[p].values() for m in ms)
                return (-ok[p], -pubs, len(p), p)
            d.prefix = sorted(ok, key=key)[0]
            d.found = True
            if len(ok) > 1:
                d.notes.append(f"multiple camera prefix candidates {sorted(ok)}; chose '{d.prefix}' "
                               f"(override with --prefix)")
        rm = per_prefix.get(d.prefix, {}) if d.found else {}

    # Vendor docs mix /camera/camera/* and /camera/* (and VIO under /insight/*): fill roles still empty
    # from sibling prefixes (same first path segment) or from anywhere when the role has any_prefix: true.
    def seg(p):
        return p.strip("/").split("/")[0] if p else ""
    sibling_fill: dict[str, list[Match]] = {}
    if d.found:
        for other, orm in per_prefix.items():
            if other == d.prefix:
                continue
            for role, ms in orm.items():
                if rm.get(role):
                    continue
                if roles[role].get("any_prefix") or (seg(other) == seg(d.prefix or "") and seg(other)):
                    sibling_fill.setdefault(role, []).extend(ms)
    for role in roles:
        ms = list(rm.get(role, []))
        if not ms and role in sibling_fill:
            ms = sibling_fill[role]
            d.notes.append(f"role '{role}' taken from different prefix: {sorted({m.topic for m in ms})[:2]}")
        d.matches[role] = sorted(ms, key=_rank)
    for role, spec in roles.items():
        if d.matches[role]:
            continue
        (d.missing_required if spec.get("required") else d.missing_optional).append(role)

    depth = d.matches.get("depth", [])
    d.depth_topic_present = bool(depth)
    d.depth_publishing = any(m.pub_count > 0 for m in depth)

    # global topics
    for key, g in profile.get("global_topics", {}).items():
        hit = [t for t in topics if _norm(t.name) == g["name"] and t.type == g["type"]]
        d.matches[key] = [Match(key, h.name, h.type, h.pub_count) for h in hit]
        if not hit:
            (d.missing_required if g.get("required") else d.missing_optional).append(key)

    # extras under prefix, not already matched
    if d.found and d.prefix:
        used = {m.topic for ms in d.matches.values() for m in ms}
        pre = (d.prefix or "") + "/"
        for t in topics:
            n = _norm(t.name)
            if n in used or not n.startswith(pre):
                continue
            if t.type in profile.get("extra_types", []):
                d.extras.append(Match("extra", n, t.type, t.pub_count, _is_compressed(n)))
    return d
