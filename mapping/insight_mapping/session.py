"""Session loading / manifest validation (docs/DATA_CONTRACT.md) and logical stream resolution."""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = REPO_ROOT / "schemas" / "session_manifest.schema.json"
DEFAULT_TOPIC_MAP = Path(__file__).resolve().parents[1] / "config" / "topic_map.yaml"

LOGICAL = ["rgb", "rgb_info", "depth", "depth_info", "left", "right", "left_info", "right_info",
           "imu", "vio_image", "vio_100hz", "tf", "tf_static"]
# Mandatory per DATA_CONTRACT §2 (CameraInfo/TF/IMU/VIO) — images are checked separately.
MANDATORY = ["imu", "tf_static", "tf"]


class SessionError(RuntimeError):
    pass


def validate_manifest(manifest: dict) -> list:
    """Validate against the main-owned schema. Returns list of error strings (empty = valid)."""
    import jsonschema
    schema = json.loads(SCHEMA_PATH.read_text())
    v = jsonschema.Draft202012Validator(schema)
    return [f"{'/'.join(map(str, e.path)) or '<root>'}: {e.message}" for e in sorted(v.iter_errors(manifest), key=lambda e: list(e.path))]


@dataclass
class Session:
    path: Path
    manifest: dict
    problems: list = field(default_factory=list)   # (severity, msg)
    streams: dict = field(default_factory=dict)    # logical -> topic
    stream_notes: list = field(default_factory=list)

    @property
    def bag_dir(self) -> Path:
        return self.path / self.manifest.get("storage", {}).get("bag_path", "bag")

    @property
    def session_id(self) -> str:
        return self.manifest.get("session_id", self.path.name)

    @classmethod
    def open(cls, path, topic_map: Optional[Path] = None, strict: bool = False) -> "Session":
        path = Path(path)
        mf = path / "manifest.json"
        if not mf.exists():
            raise SessionError(f"{mf} not found: not a Session directory")
        manifest = json.loads(mf.read_text())
        s = cls(path=path, manifest=manifest)
        for e in validate_manifest(manifest):
            s.problems.append(("error", f"manifest schema: {e}"))
        for sub in ("bag", "logs", "calibration"):
            if not (path / sub).is_dir():
                s.problems.append(("error" if sub == "bag" else "warn", f"missing directory {sub}/"))
        if manifest.get("session_id") and manifest["session_id"] != path.name:
            s.problems.append(("warn", f"session_id '{manifest['session_id']}' != directory name '{path.name}'"))
        state = manifest.get("recording", {}).get("state")
        if state != "finalized":
            s.problems.append(("error", f"recording.state is '{state}', contract requires 'finalized' for mapping"))
        st = manifest.get("integrity", {}).get("status")
        if st != "complete":
            s.problems.append(("warn" if st == "incomplete" else "error",
                               f"integrity.status = {st}; missing_required_streams="
                               f"{manifest.get('integrity', {}).get('missing_required_streams', [])}"))
        if strict and any(sev == "error" for sev, _ in s.problems):
            raise SessionError("; ".join(m for sev, m in s.problems if sev == "error"))
        s.resolve_streams(topic_map)
        return s

    def resolve_streams(self, topic_map: Optional[Path] = None):
        cfg = yaml.safe_load(Path(topic_map or DEFAULT_TOPIC_MAP).read_text())
        pats = cfg.get("patterns", {})
        streams = self.manifest.get("streams", [])
        by_topic = {s["topic"]: s for s in streams}
        out: dict = dict(cfg.get("overrides") or {})

        def pick(key, role_filter=None, exclude=()):
            if key in out:
                return
            p = pats.get(key, {})
            role, rx = p.get("role"), re.compile(p.get("regex", ".*"), re.I)
            cands = [s["topic"] for s in streams if (role is None or s["role"] == role)
                     and rx.search(s["topic"]) and s["topic"] not in exclude and s["topic"] not in out.values()]
            if len(cands) > 1:
                self.stream_notes.append(f"stream '{key}' ambiguous {cands}; using {cands[0]} (set overrides in topic_map.yaml)")
            if cands:
                out[key] = cands[0]

        for k in ("rgb", "depth", "left", "right", "vio_100hz"):
            pick(k)
        pick("vio_image")
        vios = [s["topic"] for s in streams if s["role"] == "vio"]
        if "vio_image" not in out and vios:
            rest = [t for t in vios if t != out.get("vio_100hz")]
            if rest:
                out["vio_image"] = rest[0]
        if "vio_image" in out and "vio_100hz" not in out and len(vios) == 1:
            self.stream_notes.append("only one VIO stream present; it is used as vio_image, no vio_100hz")
        pick("imu")
        for img, info in (("rgb", "rgb_info"), ("depth", "depth_info"), ("left", "left_info"), ("right", "right_info")):
            if img in out and info not in out:
                ns = out[img].rsplit("/", 1)[0]
                cands = [s["topic"] for s in streams if s["role"] == "camera_info" and s["topic"].startswith(ns + "/")]
                if len(cands) == 1:
                    out[info] = cands[0]
                else:
                    kw = {"rgb": "rgb|color", "depth": "depth", "left": "left", "right": "right"}[img]
                    c2 = [s["topic"] for s in streams if s["role"] == "camera_info" and re.search(kw, s["topic"], re.I)]
                    if c2:
                        out[info] = c2[0]
        for s in streams:
            if s["role"] in ("tf", "tf_static"):
                out.setdefault(s["role"], s["topic"])
        self.streams = {k: v for k, v in out.items() if k in LOGICAL and (v in by_topic or k in cfg.get("overrides", {}))}

    def stream_meta(self, logical: str) -> Optional[dict]:
        t = self.streams.get(logical)
        return next((s for s in self.manifest.get("streams", []) if s["topic"] == t), None)

    def missing_mandatory(self) -> list:
        miss = [k for k in MANDATORY if k not in self.streams]
        if "vio_image" not in self.streams:
            miss.append("vio")
        if not any(k in self.streams for k in ("rgb", "left")):
            miss.append("image(rgb|left)")
        for img in ("rgb", "depth", "left", "right"):
            if img in self.streams and f"{img}_info" not in self.streams:
                miss.append(f"{img}_info")
        return miss
