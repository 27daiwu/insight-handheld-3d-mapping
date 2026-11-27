"""Render and validate the unified OFFICIAL-vs-MEASURED data description (topics_v2.1.5.yaml: spec).

The tables inside DATA_CONTRACT.md between `<!-- BEGIN GENERATED:<name> -->` / `<!-- END GENERATED:<name> -->` markers are produced
here, so the document cannot drift from the YAML. `validate_spec` enforces the promotion rule: a vendor-document statement can never,
on its own, become V (verified) or O (observed)."""
from __future__ import annotations

import re
from pathlib import Path

from .config import load_profile

REPO_CAPTURE = Path(__file__).resolve().parent.parent
FIELDS = ("topic", "type", "resolution", "encoding", "nominal", "rate", "unit")
STATUS = {"V", "O", "?", "-"}
DOC_SOURCES = {"D0", "D1"}
ROLE_ORDER = ("rgb", "stereo_left", "stereo_right", "depth", "imu", "vio", "rgb_info", "stereo_left_info", "stereo_right_info", "depth_info")
SHORT = {"sensor_msgs/msg/": "", "geometry_msgs/msg/": "", "tf2_msgs/msg/": "", "std_msgs/msg/": ""}


def load_spec(path=None) -> dict:
    return load_profile(path)["spec"]


def short_type(t):
    for k in SHORT:
        if t and t.startswith(k):
            return t[len(k):]
    return t or "-"


def _s(v):
    return "-" if v is None else str(v)


def status_str(st: dict, fields=FIELDS) -> str:
    return " ".join(f"{f}:{st[f]}" for f in fields if f in st)


def _unit(u, st):
    return "-" if (u is None or st == "-") else f"{u} [{st}]"


def _hz(v):
    return "-" if v is None else f"{v:g}"


def render_unified(spec: dict) -> str:
    """Section 5a: one row per role, measured topic first, official nominal next to the delivered rate."""
    h = ("| role | topic (fw 2.1.5, measured) | message type | W x H (measured) | encoding | nominal Hz | delivered Hz run A / run B | "
         "count loss % A / B | unit | status per field |\n|---|---|---|---|---|---|---|---|---|---|")
    rows = [h]
    for r in ROLE_ORDER:
        e = spec["roles"][r]; m = e["measured"]
        if m is None:
            rows.append(f"| `{r}` | *(no such topic on fw 2.1.5)* | - | - | - | - | - | - | - | {status_str(e['status'], ('topic',))} |")
            continue
        mr, ls = m.get("mean_rate_hz"), m.get("loss_pct")
        rows.append(f"| `{r}` | `{m['topic']}` | {short_type(m['type'])} | {_s(m.get('resolution_wxh'))} | {_s(m.get('encoding'))} | {_hz(m.get('nominal_hz'))} [{e['status']['nominal']}] | "
                    f"{_hz(mr['A'])} / {_hz(mr['B'])} | {_hz(ls['A'])} / {_hz(ls['B'])} | {_unit(m.get('unit'), e['status']['unit'])} | {status_str(e['status'])} |")
    g = spec["global_topics"]
    for k in ("tf_static", "static_tf", "tf"):
        e, m = g[k], g[k]["measured"]
        if m:
            rows.append(f"| `{k}` | `{m['topic']}` | {short_type(m['type'])} | - | - | - | latched, {m['messages_per_run']} msg/run | - | - | {status_str(e['status'], ('topic', 'type', 'rate', 'unit'))} |")
        else:
            rows.append(f"| `{k}` | *(not published on fw 2.1.5)* | - | - | - | - | - | - | - | topic:{e['status']['topic']} |")
    for k, e in spec["extras_measured_only"].items():
        mr = e["mean_rate_hz"]
        rows.append(f"| `{k}` (extra) | `{e['topic']}` | {short_type(e['type'])} | - | - | {_hz(e['nominal_hz'])} | {('-' if mr is None else _hz(mr['A']) + ' / ' + _hz(mr['B']))} | - | - | {status_str(e['status'], ('topic', 'type', 'rate'))} |")
    return "\n".join(rows)


def render_official(spec: dict) -> str:
    """Section 5b: what the two guide versions say, side by side. Documentation claims only - no status column on purpose."""
    h = ("| role | v1.2.0 topic | v1.2.0 res / enc / Hz / unit | v1.2.1 topic | v1.2.1 Hz / unit | measured topic |\n|---|---|---|---|---|---|")
    rows = [h]
    for r in ROLE_ORDER:
        e = spec["roles"][r]; a, b, m = e["official_v1_2_0"], e["official_v1_2_1"], e["measured"]
        if a is None:
            rows.append(f"| `{r}` | *(not listed)* | - | *(not listed)* | - | {('`' + m['topic'] + '`') if m else '*(none)*'} |")
            continue
        rows.append(f"| `{r}` | `{a['topic']}` | {_s(a['resolution'])} / {_s(a['encoding'])} / {_hz(a['rate_hz'])} / {_s(a['unit'])} | `{b['topic']}` | "
                    f"{_hz(b['rate_hz'])} / {_s(b['unit'])} | `{m['topic']}` |")
    g = spec["global_topics"]
    for k in ("tf_static", "static_tf"):
        a, b, m = g[k]["official_v1_2_0"], g[k]["official_v1_2_1"], g[k]["measured"]
        rows.append(f"| `{k}` | {('`' + a['topic'] + '`') if a else '*(not listed)*'} | - | {('`' + b['topic'] + '`') if b else '*(not listed)*'} | - | "
                    f"{('`' + m['topic'] + '`') if m else '*(not published)*'} |")
    return "\n".join(rows)


def render_deviations(spec: dict) -> str:
    rows = ["| id | difference | official statement | measured on fw 2.1.5 | status | note |\n|---|---|---|---|---|---|"]
    for k, d in spec["deviations"].items():
        rows.append(f"| {k} | {d['title']} | {d['official']} | {d['measured']} | {d['status']} | {d['note']} |")
    return "\n".join(rows)


def render_sources(spec: dict) -> str:
    rows = ["| code | kind | what | file |\n|---|---|---|---|"]
    for k, s in spec["sources"].items():
        rows.append(f"| {k} | {s['kind']} | {s['what']} | `{s['path']}` |")
    for k, r in spec["runs"].items():
        rows.append(f"| run {k} | measurement | {r['started']}, {r['duration_s']} s, {r['note']} | `{r['raw']}` |")
    return "\n".join(rows)


BLOCKS = {"unified": render_unified, "official": render_official, "deviations": render_deviations, "sources": render_sources}


def render_all(spec=None) -> dict:
    spec = spec or load_spec()
    return {k: f(spec) for k, f in BLOCKS.items()}


def marker(name, end=False):
    return f"<!-- {'END' if end else 'BEGIN'} GENERATED:{name} -->"


def extract_block(md: str, name: str) -> str | None:
    m = re.search(re.escape(marker(name)) + r"\n(.*?)\n?" + re.escape(marker(name, True)), md, re.S)
    return m.group(1) if m else None


def inject(md: str, blocks: dict) -> str:
    for name, text in blocks.items():
        pat = re.compile(re.escape(marker(name)) + r"\n(?:.*?\n)?" + re.escape(marker(name, True)), re.S)
        rep = f"{marker(name)}\n{text}\n{marker(name, True)}"
        if not pat.search(md):
            raise ValueError(f"marker block {name} missing in document")
        md = pat.sub(lambda _m: rep, md)
    return md


def validate_spec(spec: dict, capture_dir: Path = REPO_CAPTURE) -> list[str]:
    """Return a list of rule violations (empty = ok)."""
    errs = []
    known = set(spec["sources"])
    for k, s in spec["sources"].items():
        if not (capture_dir / s["path"]).exists():
            errs.append(f"source {k}: file missing: {s['path']}")
    for r, a in spec["runs"].items():
        if not (capture_dir / a["raw"]).exists():
            errs.append(f"run {r}: raw file missing: {a['raw']}")
    entries = {f"role {r}": (e["status"], e["sources"]) for r, e in spec["roles"].items()}
    entries.update({f"global {k}": (e["status"], e["sources"]) for k, e in spec["global_topics"].items()})
    for name, (st, src) in entries.items():
        for f, v in st.items():
            if v not in STATUS:
                errs.append(f"{name}.{f}: bad status {v!r}")
            codes = src.get(f, [])
            if any(c not in known for c in codes):
                errs.append(f"{name}.{f}: unknown source code in {codes}")
            if v in ("V", "O") and not [c for c in codes if c not in DOC_SOURCES]:
                errs.append(f"{name}.{f}: status {v} but only vendor-document sources {codes}")
            if v == "V" and "?" in v:
                errs.append("unreachable")
    for k, d in spec["deviations"].items():
        if d["status"] not in STATUS:
            errs.append(f"{k}: bad status {d['status']!r}")
        if d["status"] in ("V", "O") and not [c for c in d["sources"] if c not in DOC_SOURCES]:
            errs.append(f"{k}: status {d['status']} with only vendor-document sources")
    for k, e in spec["extras_measured_only"].items():
        if not [c for c in e["sources"] if c not in DOC_SOURCES]:
            errs.append(f"extra {k}: no measurement source")
    return errs
