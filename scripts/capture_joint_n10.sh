#!/usr/bin/env bash
set -euo pipefail
joint_output=${1:?Usage: sudo bash capture_joint_n10.sh NEW_OUTPUT_DIRECTORY}
joint_tools=$(cd "$(dirname "$0")" && pwd)
test "$(id -u)" = 0 || { echo 'Run with sudo (IMU device is root-only)' >&2; exit 1; }
test ! -e "$joint_output" || { echo 'Output must be new' >&2; exit 1; }
test -e /sys/bus/i2c/devices/7-003b/driver
mkdir -p "$joint_output"
joint_pid=''
finish_joint() {
    if [[ -n "$joint_pid" ]] && kill -0 "$joint_pid" 2>/dev/null; then
        kill -TERM "$joint_pid"
        wait "$joint_pid" || true
    fi
}
trap finish_joint EXIT
for joint_device in 4 5; do
    v4l2-ctl -d "/dev/video$joint_device" --info | grep -q 'vi-output, zedx'
    v4l2-ctl -d "/dev/video$joint_device" \
        --set-fmt-video=width=1920,height=1200,pixelformat=BA10 \
        --set-ctrl=sensor_mode=0,frame_rate=30000000,exposure=10000,gain=100,override_enable=1
done
"$joint_tools/capture_imu_n10" "$joint_output/imu" 45 &
joint_pid=$!
sleep 2
kill -0 "$joint_pid"
"$joint_tools/capture_timestamps" "$joint_output/camera" 1212 60 20
wait "$joint_pid"
joint_pid=''
python3 "$joint_tools/export_timestamped_pairs.py" "$joint_output/camera" "$joint_output/images" --pairs 5 --tolerance-us 1000
python3 "$joint_tools/preview_zed_raw.py" "$joint_output/images"
python3 "$joint_tools/analyze_joint_n10.py" "$joint_output"
