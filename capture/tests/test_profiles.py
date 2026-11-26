"""Profile governance: history kept, baseline = 2.1.5, no bulk 'verified'."""
from pathlib import Path

import yaml

from insight_capture.config import DEFAULT_PROFILE, load_profile

CFG = Path(__file__).resolve().parents[1] / "config"


def test_default_is_215_and_history_is_kept():
    assert DEFAULT_PROFILE.name == "topics_v2.1.5.yaml"
    p215, p214 = load_profile(), load_profile(CFG / "topics_v2.1.4.yaml")
    assert p215["firmware_baseline"] == "2.1.5" and p214["firmware_baseline"] == "2.1.4"
    assert p214["verified"] is False and "infra1" not in p214["roles"]["stereo_left"]["suffix"]   # untouched history


def test_file_level_verified_is_false_while_any_required_role_is_unverified():
    p = load_profile()
    roles = p["roles"]
    unverified_required = [r for r, s in roles.items() if s.get("required") and not s["verification"]["verified"]]
    assert unverified_required, "test premise: some required role (vio) is not verified yet"
    assert p["verified"] is False


def test_every_role_has_evidence_and_verified_roles_have_no_blocking_numbers():
    p = load_profile()
    for r, s in p["roles"].items():
        v = s["verification"]
        assert isinstance(v["verified"], bool) and v["evidence"] and isinstance(v["pending"], list), r
    # roles known to violate the policy must NOT be marked verified
    assert p["roles"]["depth"]["verification"]["verified"] is False       # 12-13 % shortfall
    assert p["roles"]["vio"]["verification"]["verified"] is False         # pooled loss > 0.1 %
    assert p["roles"]["depth_info"]["verification"]["verified"] is False  # topic absent on 2.1.5
    assert p["global_topics"]["tf_static"]["verification"]["verified"] is False


def test_policy_is_per_role_not_bulk():
    p = load_profile()
    flags = {r: s["verification"]["verified"] for r, s in p["roles"].items()}
    assert any(flags.values()) and not all(flags.values())
