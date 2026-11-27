#!/usr/bin/env python3
"""Re-render the generated tables inside DATA_CONTRACT.md from config/topics_v2.1.5.yaml (spec:).  --check = exit 1 if stale."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from insight_capture import contract  # noqa: E402

p = Path(__file__).resolve().parents[1] / "DATA_CONTRACT.md"
old = p.read_text(encoding="utf-8")
new = contract.inject(old, contract.render_all())
if "--check" in sys.argv:
    sys.exit(0 if new == old else 1)
p.write_text(new, encoding="utf-8")
print("DATA_CONTRACT.md tables re-rendered" if new != old else "already up to date")
