#!/usr/bin/env bash
set -euo pipefail
imu_package=$(cd "$(dirname "$0")/.." && pwd)
imu_action=${1:---check}
[[ "$imu_action" = --check || "$imu_action" = --install ]] || exit 2
[[ $(uname -m) = aarch64 && $(uname -r) = 6.8.12-rt-tegra ]] || { echo 'Wrong kernel/platform' >&2; exit 1; }
tr '\0' '\n' < /sys/firmware/devicetree/base/rb,dtbbuildversion | grep -q B25EJ21_007
test -f /home/nvidia/load_zedx_n10.sh
test -d /home/nvidia/zedx-shw5g-reference
[[ $(sha256sum /lib/modules/6.8.12-rt-tegra/updates/drivers/bmi088/bmi088.ko | cut -d' ' -f1) = fcc7467d04144892add17ec84bef38e54ee78f70e45897c8886f44c776d7c97b ]]
[[ $(sha256sum /lib/modules/6.8.12-rt-tegra/extra/zedx-n10/sl_max96712.ko | cut -d' ' -f1) = 0aed5386498bade6bc750a1d87c7a36cba80dcb719b2bccf0085aa8a2e802b92 ]]
(cd "$imu_package" && sha256sum -c RUNTIME-SHA256SUMS.txt)
if [[ "$imu_action" = --check ]]; then echo 'Preflight passed; no system changes.'; exit 0; fi
[[ $(id -u) = 0 ]] || { echo 'Use sudo for installation' >&2; exit 1; }
imu_install=/opt/zedx-n10-imu-20260916
test ! -e "$imu_install" || { echo "Already exists: $imu_install; refusing overwrite" >&2; exit 1; }
mkdir "$imu_install"
cp -a "$imu_package/driver" "$imu_package/scripts" "$imu_package/source" "$imu_package/RUNTIME-SHA256SUMS.txt" "$imu_install/"
chown -R root:root "$imu_install"
chmod -R go-w "$imu_install"
(cd "$imu_install" && sha256sum -c RUNTIME-SHA256SUMS.txt)
echo "Installed to $imu_install. Original kernel modules and boot files unchanged."
echo "Next: sudo bash $imu_install/scripts/load_zedx_imu_n10.sh --full"
