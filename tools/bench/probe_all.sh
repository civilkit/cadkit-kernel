#!/bin/sh
# usage: tools/bench/probe_all.sh [id ...]   probe each BenchCAD batch-3 part (errors, box, volume, validate)
cd "$(dirname "$0")/../.." || exit 1
IDS=${*:-"bucket_003137 ball_knob_007575 clevis_014687 hinge_000079 j_hook_001383 gridfinity_bin_000166 cam_012392 motor_end_cap_000242 cable_routing_panel_004961 helical_gear_015542"}
for f in $IDS; do
  echo "-- $f"
  node tools/bench/probe.mjs tools/bench/benchcad/$f.ckpart.json | cut -c1-400
done
