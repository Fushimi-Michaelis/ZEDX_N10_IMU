#!/usr/bin/env bash
# Build the parallel SHW RGB and official ZED RGB-D capture workers.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
sdk=${ZED_DIR:-$root/evidence/zed-sdk-5.4/extracted}
deps="$(dirname "$sdk")/deps/usr/lib/aarch64-linux-gnu"
[[ $(uname -m) = aarch64 ]] || { echo 'Build on Thor' >&2; exit 1; }
[[ -f "$sdk/include/sl/Camera.hpp" && -f "$sdk/lib/libsl_zed.so" ]] || {
    echo 'Set ZED_DIR to the official L4T38.2 SDK directory' >&2; exit 1;
}
mkdir -p "$root/build"
g++ -std=c++17 -O2 -Wall -Wextra "$root/source/capture_shw_n10.cpp" \
    -I/usr/src/jetson_multimedia_api/argus/include \
    -L/usr/lib/aarch64-linux-gnu/nvidia -lnvargus_socketclient \
    -o "$root/build/capture_shw_n10"
g++ -std=c++17 -O2 -Wall -Wextra -pthread "$root/source/capture_zed_n10.cpp" \
    -isystem "$sdk/include" -isystem /usr/local/cuda/include -L"$sdk/lib" \
    -Wl,-rpath,"$sdk/lib:$deps" -Wl,-rpath-link,"$deps" -lsl_zed -o "$root/build/capture_zed_n10"
