#!/usr/bin/env bash
# Save the 2D and 3D maps after jig_path_motion logs "Path complete".
# Usage: ./save_maps.sh [output_dir]
set -e
OUT=${1:-$HOME/jig_maps/$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUT"

# 2D occupancy grid -> jig_map_2d.pgm + .yaml (Nav2 format)
ros2 run nav2_map_server map_saver_cli -f "$OUT/jig_map_2d" \
  --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true

# 3D point cloud map (optimized poses) -> .ply, straight from the RTAB-Map database.
# Run while rtabmap is still up, or after stopping it (db at ~/.ros/rtabmap.db).
rtabmap-export --scan --voxel 0.05 --output_dir "$OUT" --output jig_map_3d ~/.ros/rtabmap.db

# Keep the database too (relaunching with -d deletes it)
cp ~/.ros/rtabmap.db "$OUT/"
echo "Saved to $OUT"