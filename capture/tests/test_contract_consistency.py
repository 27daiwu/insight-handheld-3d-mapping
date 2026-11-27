"""DATA_CONTRACT.md  <->  topics_v2.1.5.yaml (spec:)  <->  archived vendor guides  <->  raw measurements  <->  tool behaviour.

Nothing here talks to a camera. Measured numbers are re-derived from the raw inspect outputs stored in reports/ (real recordings A and B)."""
import html
import json
import re
from pathlib import Path

import pytest

from insight_capture import SCHEMA_VERSION, contract
from insight_capture.config import load_profile
from insight_capture.discovery import TopicInfo, discover

CAP = Path(__file__).resolve().parents[1]
REP = CAP / "reports"
MD = (CAP / "DATA_CONTRACT.md").read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def prof():
    return load_profile()


@pytest.fixture(scope="module")
def spec(prof):
    return prof["spec"]


@pytest.fixture(scope="module")
def runs(spec):
    return {k: json.loads((CAP / r["raw"]).read_text()) for k, r in spec["runs"].items()}


# ------------------------------------------------------------------ 1. document <-> yaml
def test_spec_obeys_promotion_rules(spec):
    assert contract.validate_spec(spec) == []


def test_generated_tables_are_up_to_date(spec):
    for name, text in contract.render_all(spec).items():
        assert contract.extract_block(MD, name) == text, f"DATA_CONTRACT.md block '{name}' is stale: run tools/render_contract_tables.py"


def test_document_declares_version_status_and_schema():
    assert "DRAFT v0.3" in MD.splitlines()[0] and "Not an approved project contract" in MD
    assert f"`{SCHEMA_VERSION}`" in MD, "manifest schema_version in the contract must equal insight_capture.SCHEMA_VERSION"
    for mark in ("**[V]**", "**[O]**", "**[?]**"):
        assert mark in MD                                   # legend kept
    assert "can never be [V] or [O]" in MD                  # promotion rule is written down


def test_each_spec_role_exists_in_profile_roles(prof, spec):
    assert set(spec["roles"]) == set(prof["roles"])
    assert set(contract.ROLE_ORDER) == set(prof["roles"])


# ------------------------------------------------------------------ 2. official columns == archived guides
def _guide_rows(version):
    t = (REP / "vendor_docs" / f"looper_insight9_quickstart_v{version}.html").read_text(encoding="utf-8", errors="replace")
    rows = []
    for r in re.findall(r"<tr[^>]*>(.*?)</tr>", t, re.S):
        c = [html.unescape(re.sub(r"<[^>]+>", "", x)).strip() for x in re.findall(r"<t[dh][^>]*>(.*?)</t[dh]>", r, re.S)]
        if c and c[0].startswith("/"):
            rows.append(c)
    return {r[0]: r for r in rows}


@pytest.mark.parametrize("ver,key", [("1.2.0", "official_v1_2_0"), ("1.2.1", "official_v1_2_1")])
def test_official_columns_match_the_archived_guide_exactly(spec, ver, key):
    rows = _guide_rows(ver)
    seen = set()
    entries = [(r, e[key]) for r, e in spec["roles"].items()] + [(k, e[key]) for k, e in spec["global_topics"].items()]
    for name, o in entries:
        if o is None:
            continue
        row = rows.get(o["topic"])
        assert row, f"{key}.{name}: topic {o['topic']} is not in guide v{ver}"
        seen.add(o["topic"])
        assert row[4] == o["type"], name
        if "resolution" in o:
            assert (None if row[1] == "--" else row[1]) == o["resolution"], name
            assert (None if row[3] == "--" else row[3]) == o["encoding"], name
            hz = None if row[2] == "--" else float(row[2].rstrip("Hz"))
            assert hz == o["rate_hz"], f"{name}: guide says {row[2]}"
            assert ("mm" in row[5] or "milli" in row[5]) == (o["unit"] == "mm"), name
    assert seen == set(rows), f"guide v{ver} rows missing from the spec: {set(rows) - seen}"


def test_the_two_guides_disagree_exactly_where_the_contract_says(spec):
    a, b = _guide_rows("1.2.0"), _guide_rows("1.2.1")
    assert a["/insight/vio_pose"][2] == "30Hz" and b["/insight/vio_pose"][2] == "15Hz"
    assert "/static_tf" in a and "/static_tf" not in b and "/tf_static" in b
    assert a["/insight/depth"][2] == b["/camera/depth/image_rect_raw"][2] == "15Hz"
    assert all(k.startswith("/insight/") or k == "/static_tf" for k in a)


# ------------------------------------------------------------------ 3. measured columns == raw data
def _stats(run, name):
    return run["topics"][name], run["diagnostics"]["topics"][name]


@pytest.mark.parametrize("role", contract.ROLE_ORDER)
def test_measured_numbers_trace_to_raw_recordings(spec, runs, role):
    e = spec["roles"][role]; m = e["measured"]
    if m is None:
        for run in runs.values():
            assert not any(t["role"] == role for t in run["manifest"]["topics"])
        return
    for k, run in runs.items():
        planned = {t["role"]: t for t in run["manifest"]["topics"]}[role]
        assert planned["name"] == m["topic"] and planned["type"] == m["type"]
        st, dg = _stats(run, m["topic"])
        assert abs(st["stamp_mean_rate_hz"] - m["mean_rate_hz"][k]) < 0.0015, (role, k, st["stamp_mean_rate_hz"])
        assert abs(dg["loss_vs_reference"] * 100 - m["loss_pct"][k]) < 0.0015, (role, k, dg["loss_vs_reference"])
        assert planned["reference_hz"] == m["nominal_hz"]
        assert planned["frame_id"] == m["frame_id"]


def test_extras_and_tf_static_trace_to_raw(spec, runs):
    for name, ex in spec["extras_measured_only"].items():
        for k, run in runs.items():
            st = run["topics"][ex["topic"]]
            assert st["type"] == ex["type"]
            if ex["mean_rate_hz"]:
                assert abs(st["stamp_mean_rate_hz"] - ex["mean_rate_hz"][k]) < 0.0015
    for run in runs.values():
        assert run["topics"]["/tf_static"]["count"] == spec["global_topics"]["tf_static"]["measured"]["messages_per_run"]


def test_topics_exist_in_real_topic_list_and_absent_ones_are_absent(spec):
    tl = (REP / "camera_data" / "topics_domain9.txt").read_text().splitlines()
    names = {l.split()[0]: l.split("[")[1].rstrip("]") for l in tl if "[" in l}
    assert len(names) == 18
    for e in list(spec["roles"].values()) + [spec["global_topics"]["tf_static"]]:
        if e["measured"]:
            assert names[e["measured"]["topic"]] == e["measured"]["type"]
    for ex in spec["extras_measured_only"].values():
        assert names[ex["topic"]] == ex["type"]
    assert "/static_tf" not in names and "/tf" not in names
    assert not any(n.startswith("/insight/") for n in names)                     # no v1.2.0 namespace on fw 2.1.5
    assert not any("depth/camera_info" in n for n in names)
    assert spec["global_topics"]["static_tf"]["measured"] is None and spec["global_topics"]["tf"]["measured"] is None


def test_resolutions_match_message_facts_jpeg_and_api(spec):
    f = json.loads((REP / "contract_evidence/real_fw215_message_facts.json").read_text())
    j = json.loads((REP / "contract_evidence/jpeg_sof_check.json").read_text())["decoded_2026-11-27"]
    wxh = lambda w, h: f"{w}x{h}"
    R = {r: spec["roles"][r]["measured"] for r in ("rgb", "stereo_left", "stereo_right", "depth", "rgb_info", "stereo_left_info", "stereo_right_info")}
    assert R["rgb"]["resolution_wxh"] == wxh(j["width_px"], j["height_px"]) == wxh(f["color_camera_info"]["w"], f["color_camera_info"]["h"])
    assert R["rgb_info"]["resolution_wxh"] == R["rgb"]["resolution_wxh"]
    for r, key in (("stereo_left", "infra1_image"), ("stereo_right", "infra2_image")):
        assert R[r]["resolution_wxh"] == wxh(f[key]["w"], f[key]["h"]) and R[r]["encoding"] == f[key]["enc"]
    assert R["depth"]["resolution_wxh"] == wxh(f["depth"]["w"], f["depth"]["h"]) and R["depth"]["encoding"] == f["depth"]["enc"] == "mono16"
    assert R["stereo_left_info"]["resolution_wxh"] == wxh(f["infra1_camera_info"]["w"], f["infra1_camera_info"]["h"])
    api = json.loads((REP / "camera_data/stereo-format.json").read_text())["data"]["format"]
    assert api.startswith("mono8 544x640") and R["stereo_left"]["resolution_wxh"] in api
    api = json.loads((REP / "camera_data/camera-format.json").read_text())["data"]["format"]
    assert R["rgb"]["resolution_wxh"] in api and api.startswith("mjpg")


def test_guide_resolution_is_the_reverse_of_the_measured_one(spec):
    for r in ("rgb", "stereo_left", "stereo_right", "depth"):
        w, h = spec["roles"][r]["measured"]["resolution_wxh"].split("x")
        for key in ("official_v1_2_0", "official_v1_2_1"):
            assert spec["roles"][r][key]["resolution"] == f"{h}x{w}"


# ------------------------------------------------------------------ 4. the three differences the project must not lose
def test_depth_difference_is_recorded_and_not_softened(spec):
    d = spec["roles"]["depth"]; mm = d["measured"]
    assert d["official_v1_2_0"]["rate_hz"] == d["official_v1_2_1"]["rate_hz"] == 15
    assert d["official_v1_2_0"]["encoding"] == "mono16" and d["official_v1_2_0"]["unit"] == "mm"
    assert all(12.0 < v < 13.5 for v in mm["loss_pct"].values()) and all(13.0 < v < 13.2 for v in mm["mean_rate_hz"].values())
    assert d["status"]["rate"] == "V" and d["status"]["unit"] == "?" and d["status"]["nominal"] == "O"   # unit is documentation only
    assert "DEV-DEPTH-RATE" in d["deviations"] and "DEV-DEPTH-UNIT" in d["deviations"]
    for token in ("12.654", "12.828", "13.1", "13.077"):
        assert token in MD


def test_vio_difference_is_recorded_for_both_guides(spec):
    v = spec["roles"]["vio"]
    assert (v["official_v1_2_0"]["rate_hz"], v["official_v1_2_1"]["rate_hz"]) == (30, 15)
    assert 99.5 < min(v["measured"]["mean_rate_hz"].values()) and max(v["measured"]["mean_rate_hz"].values()) < 100.5
    assert spec["extras_measured_only"]["vio_image"]["nominal_hz"] == 30
    assert v["status"]["nominal"] != "V" and v["status"]["unit"] == "?"                                  # nominal only from the topic name
    assert "equivalence" in spec["deviations"]["DEV-VIO-RATE"]["note"] and "UNCONFIRMED" in spec["deviations"]["DEV-VIO-RATE"]["note"]
    assert "DEV-VIO-RATE" in v["deviations"] and "30 Hz (v1.2.0)" in MD and "15 Hz (v1.2.1)" in MD


def test_static_tf_difference_is_recorded(spec):
    g = spec["global_topics"]
    assert g["tf_static"]["official_v1_2_0"]["topic"] == "/static_tf" and g["tf_static"]["official_v1_2_1"]["topic"] == "/tf_static"
    assert g["tf_static"]["measured"]["topic"] == "/tf_static" and g["static_tf"]["measured"] is None
    assert "`/static_tf`" in MD and "`/tf_static`" in MD and "DEV-STATIC-TF-NAME" in MD


def test_no_status_was_raised_from_documentation(spec):
    for name, e in list(spec["roles"].items()) + list(spec["global_topics"].items()):
        for f, st in e["status"].items():
            if st in ("V", "O"):
                assert [c for c in e["sources"].get(f, []) if c not in contract.DOC_SOURCES], (name, f)
    assert spec["roles"]["depth_info"]["official_v1_2_0"] is None and spec["roles"]["depth_info"]["official_v1_2_1"] is None


# ------------------------------------------------------------------ 5. spec <-> profile verification policy <-> discovery
def test_verified_roles_have_measured_topic_type_and_rate(prof, spec):
    for r, role in prof["roles"].items():
        st = spec["roles"][r]["status"]
        if role["verification"]["verified"]:
            assert st["topic"] == st["type"] == st["rate"] == "V", r
    assert prof["verified"] is False                          # file-level flag untouched by this document change


def test_discovery_resolves_every_measured_topic_from_the_real_topic_list(prof, spec):
    tl = (REP / "camera_data" / "topics_domain9.txt").read_text().splitlines()
    infos = [TopicInfo(l.split()[0], l.split("[")[1].rstrip("]"), 1) for l in tl if "[" in l]
    d = discover(infos, prof)
    chosen = {r: [m.topic for m in ms] for r, ms in d.chosen("raw").items() if ms}
    for r, e in spec["roles"].items():
        if e["measured"]:
            assert chosen[r] == [e["measured"]["topic"]], r
        else:
            assert r not in chosen
    assert d.prefix == "/camera/camera"


def test_v120_names_are_documented_limitation_not_silently_supported(prof):
    """Contract section 9 #10: with the v1.2.0 /insight/* names discovery finds imu + vio only. If aliases are added later
    (after the names are observed on a device) this test and the contract must change together."""
    rows = [r for r in _guide_rows("1.2.0").values()]
    d = discover([TopicInfo(r[0], r[4], 1) for r in rows], prof)
    found = {r for r, ms in d.matches.items() if ms}
    assert found == {"imu", "vio"}
    assert {"rgb", "stereo_left", "stereo_right", "tf_static"} <= set(d.missing_required)
    assert "not** the v1.2.0 image/depth/camera_info/`/static_tf` names" in MD


# ------------------------------------------------------------------ 6. references elsewhere agree
def test_current_documents_point_to_v03_and_use_the_unified_definitions():
    readme = (CAP / "README.md").read_text()
    assert "DATA_CONTRACT.md" in readme and "v0.3" in readme
    yml = (CAP / "config/topics_v2.1.5.yaml").read_text()
    stale = re.compile(r"0\.15\s*[-–]\s*0\.4|pooled loss[^\n]*0\.15")
    assert not stale.search(yml) and not stale.search(readme)
    body = MD.split("## 10. Change log")[0]
    assert not stale.search(body)                           # only the change log may quote the old gap-based number
    assert "delivered Hz" in MD and "count loss %" in MD


def test_acceptance_documents_carry_the_correction():
    for f, tokens in (("P1_acceptance_report.md", ("v0.3", "count")), ("4_topic_config_changelog.md", ("v0.3", "spec")),
                      ("3_depth_loss_root_cause.md", ("v0.3",)), ("5_next_phase_go_no_go.md", ("v0.3",))):
        t = (REP / f).read_text()
        for tok in tokens:
            assert tok in t, (f, tok)


def test_raw_evidence_and_history_are_preserved():
    for rel in ("camera_data/probe_60s_run1.json", "camera_data/real5min_inspect.json", "camera_data/real5min_v215_inspect.json",
                "camera_param_experiment/camera_param_experiment.json", "camera_param_experiment/backup_original_20261126_181847.json",
                "timesyncd_experiment/timesyncd_experiment.json", "timesyncd_experiment/run1_INVALID_B_reactivated.json",
                "depth_loss/layer_compare.json", "vendor_docs/looper_insight9_quickstart_v1.2.0.html",
                "vendor_docs/looper_insight9_quickstart_v1.2.1.html", "2_wsl2_packet_loss_rca.md", "1_camera_access_report.md",
                "3_capture_tool_test_report.md"):
        assert (REP / rel).exists(), rel
    assert (CAP / "config/topics_v2.1.4.yaml").exists()
    sums = (REP / "vendor_docs/SHA256SUMS").read_text()
    import hashlib
    for line in sums.splitlines():
        digest, path = line.split()
        assert hashlib.sha256((REP / "vendor_docs" / path).read_bytes()).hexdigest() == digest, path
