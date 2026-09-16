#!/usr/bin/env bash
# Ordered initialization. Original NVIDIA module stays untouched on disk.
set -euo pipefail
imu_root=$(cd "$(dirname "$0")/.." && pwd)
imu_mode=${1:---full}
[[ "$imu_mode" = --full || "$imu_mode" = --imu-only || "$imu_mode" = --rollback ]] || exit 2
[[ $(id -u) = 0 && $(uname -r) = 6.8.12-rt-tegra ]] || { echo 'Requires root and verified N10 kernel' >&2; exit 1; }
tr '\0' '\n' < /sys/firmware/devicetree/base/rb,dtbbuildversion | grep -q B25EJ21_007
if fuser /dev/video4 /dev/video5 /dev/spsc_bmi0 2>/dev/null; then
    echo 'Stop all camera/IMU consumers before reload.' >&2; exit 1
fi
# Refuse to detach an IMU outside the one included in this task.
for imu_bound in /sys/bus/i2c/drivers/bmi088/[0-9]*-*; do
    [[ -L "$imu_bound" ]] || continue
    [[ $(basename "$imu_bound") = 7-003b ]] || { echo "Other IMU is bound: $imu_bound" >&2; exit 1; }
done
if [[ -e /sys/class/bmi_spsc/spsc_bmi0/timer_control ]]; then
    echo 0 > /sys/class/bmi_spsc/spsc_bmi0/timer_control
fi
if [[ -d /sys/module/bmi088 ]]; then rmmod bmi088; fi
if [[ -d /sys/module/bmi_spsc ]]; then rmmod bmi_spsc; fi
imu_vendor=/lib/modules/6.8.12-rt-tegra/updates/drivers/bmi088/bmi088.ko
if [[ "$imu_mode" = --rollback ]]; then
    insmod "$imu_vendor"
    echo 'Restored original NVIDIA BMI088 module; ZED IMU data service is unavailable in that configuration.'
    exit 0
fi
restore_imu_on_error() {
    if [[ -d /sys/module/bmi088 ]]; then rmmod bmi088 || true; fi
    if [[ -d /sys/module/bmi_spsc ]]; then rmmod bmi_spsc || true; fi
    insmod "$imu_vendor" || true
}
trap restore_imu_on_error ERR
if [[ "$imu_mode" = --full ]]; then
    bash /home/nvidia/load_zedx_n10.sh
fi
test -e /sys/bus/i2c/devices/7-0062/driver
insmod "$imu_root/driver/bmi_spsc.ko"
insmod "$imu_root/driver/bmi088.ko"
test -e /sys/bus/i2c/devices/7-003b/driver
test -e /dev/spsc_bmi0
[[ $(cat /sys/class/bmi_spsc/spsc_bmi0/i2c_bus) = 7 ]]
[[ $(cat /sys/class/bmi_spsc/spsc_bmi0/acc_addr) = 0x3c ]]
[[ $(cat /sys/class/bmi_spsc/spsc_bmi0/gyro_addr) = 0x3b ]]
test -e /sys/class/bmi_spsc/spsc_bmi0/ring_drops
chmod 600 /dev/spsc_bmi0
trap - ERR
echo 'ZED X IMU ready; timer is stopped until capture starts.'
