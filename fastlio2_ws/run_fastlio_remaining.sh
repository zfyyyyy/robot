#!/usr/bin/env bash
set -eo pipefail
WS="$HOME/Desktop/fastlio2_ws"
BAG_DIR="$WS/bags/r3live"
RESULT_DIR="$WS/results/fastlio_batch_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULT_DIR"
cd "$WS" || exit 1
source /opt/ros/noetic/setup.bash
source devel/setup.bash

run_one() {
  local name="$1"
  local rate="$2"
  local bag="$BAG_DIR/${name}.bag"
  local out_dir="$RESULT_DIR/$name"
  mkdir -p "$out_dir"
  echo "===== [$name] start $(date '+%F %T') rate=$rate =====" | tee -a "$RESULT_DIR/batch.log"
  if [ ! -f "$bag" ]; then
    echo "[$name] missing bag: $bag" | tee -a "$RESULT_DIR/batch.log"
    return 1
  fi

  rm -f "$WS/src/FAST_LIO/PCD/scans.pcd"

  roslaunch fast_lio mapping_avia_easy.launch rviz:=false save_pcd:=true > "$out_dir/roslaunch.log" 2>&1 &
  local launch_pid=$!
  echo "$launch_pid" > "$out_dir/roslaunch.pid"
  sleep 8

  rosbag play "$bag" -r "$rate" > "$out_dir/rosbag.log" 2>&1
  local bag_status=$?
  echo "[$name] rosbag finished status=$bag_status $(date '+%F %T')" | tee -a "$RESULT_DIR/batch.log"

  kill -INT "$launch_pid" 2>/dev/null || true
  sleep 12
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -TERM "$launch_pid" 2>/dev/null || true
    sleep 4
  fi
  wait "$launch_pid" 2>/dev/null || true

  if [ -f "$WS/src/FAST_LIO/PCD/scans.pcd" ]; then
    mv "$WS/src/FAST_LIO/PCD/scans.pcd" "$out_dir/${name}_scans.pcd"
    du -h "$out_dir/${name}_scans.pcd" | tee -a "$RESULT_DIR/batch.log"
  else
    echo "[$name] WARNING: scans.pcd not found after run" | tee -a "$RESULT_DIR/batch.log"
  fi

  cp "$WS/src/FAST_LIO/config/avia.yaml" "$out_dir/avia.yaml" 2>/dev/null || true
  cp "$WS/src/FAST_LIO/launch/mapping_avia_easy.launch" "$out_dir/mapping_avia_easy.launch" 2>/dev/null || true
  echo "===== [$name] end $(date '+%F %T') =====" | tee -a "$RESULT_DIR/batch.log"
}

run_one "degenerate_seq_02" 0.5
run_one "hku_park_01" 0.5
run_one "hku_main_building" 0.5

echo "ALL_DONE $RESULT_DIR" | tee -a "$RESULT_DIR/batch.log"
