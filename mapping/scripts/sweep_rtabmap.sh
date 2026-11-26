#!/usr/bin/env bash
# usage: sweep_rtabmap.sh <session> <work> <outroot> <gt.npz> <backend> <sensor> "name|--set k=v --set ..." ...
# Runs variants in parallel (distinct ROS_DOMAIN_ID each) and prints ATE per variant.
set -u
SESSION=$1; WORK=$2; OUT=$3; GT=$4; BACKEND=$5; SENSOR=$6; shift 6
PY="${PY:-.venv/bin/python}"
i=10
for v in "$@"; do
  name="${v%%|*}"; args="${v#*|}"; i=$((i+1))
  ( ROS_DOMAIN_ID=$i $PY -m insight_mapping.cli backend $BACKEND $SESSION --work $WORK --out $OUT/$name --sensor $SENSOR $args > $OUT/$name.log 2>&1 ) &
done
wait
$PY - <<PYEOF
import sys, glob, os, numpy as np
from insight_mapping import compare
gt = compare.load_gt("$GT")
for d in sorted(glob.glob("$OUT/*/")):
    c = d + "trajectory_opt.csv"
    if not os.path.exists(c): print(os.path.basename(d.rstrip('/')), "FAILED"); continue
    m, _ = compare.traj_metrics(c, gt, 1)
    print(f"{os.path.basename(d.rstrip('/')):24s} ATE_al={m['ate_aligned']['rmse_m']:.4f} anch={m['ate_anchored']['rmse_m']:.4f} RPEt={m['rpe']['trans_rmse_m']:.4f} RPEr={m['rpe']['rot_rmse_deg']:.2f} frames={m['frames']}")
PYEOF
