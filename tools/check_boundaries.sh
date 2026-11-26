#!/usr/bin/env bash
# Verify a feature branch only touches its own directory.
# Usage: tools/check_boundaries.sh <capture|mapping> [base-ref=main]
set -euo pipefail
side="${1:?usage: $0 <capture|mapping> [base-ref]}"
base="${2:-main}"
case "$side" in
  capture|mapping) ;;
  *) echo "side must be capture or mapping" >&2; exit 2 ;;
esac
bad=$(git diff --name-only "$base"...HEAD | grep -v "^${side}/" || true)
if [ -n "$bad" ]; then
  echo "Boundary violation: branch touches files outside ${side}/:" >&2
  echo "$bad" >&2
  exit 1
fi
echo "OK: all changes are within ${side}/"
