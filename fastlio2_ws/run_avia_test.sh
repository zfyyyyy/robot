#!/usr/bin/env bash
set -eo pipefail
WS="$HOME/Desktop/fastlio2_ws"
PKG="$WS/src/FAST_LIO"
BAG="$WS/bags/r3live/degenerate_seq_02.bag"
PCD_DIR="$PKG/PCD"
OUT="$PCD_DIR/scans_avia_test.pcd"
RUN_DIR="$WS/results/avia_test_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR" "$PCD_DIR"
cd "$WS"
source /opt/ros/noetic/setup.bash
source devel/setup.bash

pkill -f "rosbag play" || true
pkill -f fastlio_mapping || true
pkill -f pcd_to_pointcloud || true
pkill -f rviz || true
sleep 2
rm -f "$PCD_DIR/scans.pcd"

echo "RUN_DIR=$RUN_DIR" | tee "$RUN_DIR/status.log"
echo "CONFIG=avia" | tee -a "$RUN_DIR/status.log"
echo "BAG=$BAG" | tee -a "$RUN_DIR/status.log"
echo "RATE=2.0" | tee -a "$RUN_DIR/status.log"
echo "START $(date '+%F %T')" | tee -a "$RUN_DIR/status.log"

roslaunch fast_lio mapping_avia_easy.launch rviz:=false save_pcd:=true > "$RUN_DIR/roslaunch.log" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$RUN_DIR/roslaunch.pid"
sleep 8

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