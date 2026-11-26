The rosbag2 directories of runs R1/R2 (~5 GB) are NOT in the repo. They were moved to
`/home/hebe/insight_data/depth_loss_bags/<run>_bag_depth|bag_load`. `layer_compare.py` / `analyze_run.py` expect them under
`runs/<run>/bag_depth` (symlink or copy back to re-run). The per-consumer JSON stamps and counters.json here are sufficient to verify the numbers
for the four live readers; the rosbag-vs-reader equality in report 3 was computed from the bags before the move (`../layer_compare.json`).
