#!/usr/bin/env bash
# Export the 2D + 3D maps after a mapping run and convert them for Godot.
#
# Usage (inside the container, mapping.launch.py still running):
#   ./export_maps.sh [output_dir]
#
# Order matters:
#   1. 2D map is saved from the live /map topic        (launch must be running)
#   2. you stop the launch -> RTAB-Map writes its database on shutdown
#   3. 3D cloud is exported from the database           (launch must be stopped)
#   4. map_to_godot.py writes .glb/.png files for Godot
set -e
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-/ros2_ws/maps/$(date +%Y%m%d_%H%M%S)}
DB=${RTABMAP_DB:-$HOME/.ros/rtabmap.db}
mkdir -p "$OUT"

echo "── 1/4  Saving 2D map from /map ..."
ros2 run nav2_map_server map_saver_cli -f "$OUT/map_2d" \
  --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true

echo
read -r -p "── 2/4  Stop mapping.launch.py now (Ctrl+C in its terminal), then press Enter... "
sleep 2

echo "── 3/4  Exporting 3D cloud from $DB ..."
cp "$DB" "$OUT/rtabmap.db"
rtabmap-export --scan --voxel 0.05 --output_dir "$OUT" --output map_3d "$OUT/rtabmap.db"
PLY=$(ls -t "$OUT"/map_3d*.ply 2>/dev/null | head -n 1)
[ -n "$PLY" ] || { echo "no .ply produced — check: rtabmap-export --help"; exit 1; }

echo "── 4/4  Converting for Godot ..."
python3 "$SCRIPT_DIR/map_to_godot.py" --map2d "$OUT/map_2d.yaml" --cloud "$PLY" --out "$OUT/godot"

echo
echo "Done. Copy $OUT/godot/ into your Godot project folder."