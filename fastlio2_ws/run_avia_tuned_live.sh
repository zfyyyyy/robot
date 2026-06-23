#!/usr/bin/env bash
set -eo pipefail
WS="$HOME/Desktop/fastlio2_ws"
PKG="$WS/src/FAST_LIO"
BAG="$WS/bags/r3live/degenerate_seq_02.bag"
PCD_DIR="$PKG/PCD"
OUT="$PCD_DIR/scans_avia_tuned_test.pcd"
RUN_DIR="$WS/results/avia_tuned_live_$(date +%Y%m%d_%H%M%S)"
LAUNCH="$PKG/launch/mapping_avia_tuned_live.launch"
RVIZ="$PKG/rviz_cfg/fastlio2_live_no_decay.rviz"
mkdir -p "$RUN_DIR" "$PCD_DIR"
cd "$WS"
source /opt/ros/noetic/setup.bash
source devel/setup.bash

cp "$PKG/rviz_cfg/fastlio2_live.rviz" "$RVIZ"
python3 - <<PY
from pathlib import Path
p=Path('$RVIZ')
s=p.read_text()
s=s.replace('Decay Time: 20', 'Decay Time: 0')
p.write_text(s)
PY

cat > "$LAUNCH" <<'XML'
<launch>
  <arg name="rviz" default="true" />
  <arg name="save_pcd" default="true" />
  <rosparam command="load" file="$(find fast_lio)/config/avia.yaml" />

  <!-- Avia tuned for small/degenerate scenes: keep Avia scan_line/extrinsic, reduce blind zone and map range. -->
  <param name="preprocess/blind" type="double" value="0.5" />
  <param name="mapping/fov_degree" type="double" value="180" />
  <param name="mapping/det_range" type="double" value="100.0" />

  <param name="feature_extract_enable" type="bool" value="0"/>
  <param name="point_filter_num" type="int" value="3"/>
  <param name="max_iteration" type="int" value="3" />
  <param name="filter_size_surf" type="double" value="0.5" />
  <param name="filter_size_map" type="double" value="0.5" />
  <param name="cube_side_length" type="double" value="1000" />
  <param name="runtime_pos_log_enable" type="bool" value="0" />
  <param name="publish/path_en" type="bool" value="true" />
  <param name="pcd_save/pcd_save_en" type="bool" value="$(arg save_pcd)" />

  <node pkg="fast_lio" type="fastlio_mapping" name="laserMapping" output="screen" />
  <node pkg="fast_lio" type="thin_live_cloud.py" name="thin_live_cloud" output="screen">
    <param name="stride" value="2" />
  </node>
  <group if="$(arg rviz)">
    <node launch-prefix="nice" pkg="rviz" type="rviz" name="rviz" args="-d $(find fast_lio)/rviz_cfg/fastlio2_live_no_decay.rviz" />
  </group>
</launch>
XML

pkill -f "rosbag play" || true
pkill -f fastlio_mapping || true
pkill -f pcd_to_pointcloud || true
pkill -f rviz || true
pkill -f rqt_image_view || true
sleep 2
rm -f "$PCD_DIR/scans.pcd"

echo "RUN_DIR=$RUN_DIR" | tee "$RUN_DIR/status.log"
echo "CONFIG=avia_tuned" | tee -a "$RUN_DIR/status.log"
echo "BAG=$BAG" | tee -a "$RUN_DIR/status.log"
echo "RATE=2.0" | tee -a "$RUN_DIR/status.log"
echo "OUTPUT=$OUT" | tee -a "$RUN_DIR/status.log"
echo "START $(date '+%F %T')" | tee -a "$RUN_DIR/status.log"

export DISPLAY=:0
roslaunch fast_lio mapping_avia_tuned_live.launch rviz:=true save_pcd:=true > "$RUN_DIR/roslaunch.log" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$RUN_DIR/roslaunch.pid"
sleep 8

rqt_image_view /camera/image_color/compressed > "$RUN_DIR/rqt_image_view.log" 2>&1 &
echo $! > "$RUN_DIR/rqt_image_view.pid"
sleep 1

rosbag play "$BAG" -r 2.0 > "$RUN_DIR/rosbag.log" 2>&1
BAG_STATUS=$?
echo "ROSBAG_DONE status=$BAG_STATUS $(date '+%F %T')" | tee -a "$RUN_DIR/status.log"

kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 15
if kill -0 "$LAUNCH_PID" 2>/dev/null; then
  kill -TERM "$LAUNCH_PID" 2>/dev/null || true
  sleep 4
fi
wait "$LAUNCH_PID" 2>/dev/null || true

if [ -f "$PCD_DIR/scans.pcd" ]; then
  mv -f "$PCD_DIR/scans.pcd" "$OUT"
  ls -lh "$OUT" | tee -a "$RUN_DIR/status.log"
  echo "DONE $(date '+%F %T')" | tee -a "$RUN_DIR/status.log"
else
  echo "ERROR: scans.pcd not generated" | tee -a "$RUN_DIR/status.log"
  exit 2
fi