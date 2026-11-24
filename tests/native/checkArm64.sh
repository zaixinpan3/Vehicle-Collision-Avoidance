#!/usr/bin/env bash
# Run after exportArm64Sources, arm64ParityFixtures and buildArm64Kernels.sh.
set -euo pipefail
if [[ $# != 2 ]]; then
    echo "Usage: $0 ARM64_PACKAGE MATLAB_PARITY_FIXTURE" >&2
    exit 2
fi
dir=$(realpath "$1")
test_src=$(dirname "$(realpath "$0")")
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cc=${CC:-aarch64-linux-gnu-gcc}
compile=("$cc" -O2 -I"$dir/include" -I"$dir/include/pcbf_prediction" -I"$dir/include/pcbf_geometry")
link=(-L"$dir/lib" -Wl,--allow-shlib-undefined -lm)
"${compile[@]}" "$test_src/arm64_parity.c" "${link[@]}" -lnrmm_target -lpcbf_geometry -lpcbf_prediction -o "$work/parity"
"${compile[@]}" "$test_src/ipopt_bridge_test.c" "${link[@]}" -lpcbf_ipopt -o "$work/solver"
run_target() {
    if [[ -n ${QEMU_AARCH64:-} ]]; then
        "${QEMU_AARCH64}" -L "${ARM64_SYSROOT:?Set the target root filesystem}" \
            -E "LD_LIBRARY_PATH=$dir/lib:$ARM64_SYSROOT/usr/lib/aarch64-linux-gnu" \
            -E "CASADIPATH=$dir/lib" -E OPENBLAS_NUM_THREADS=1 "$@"
    else
        LD_LIBRARY_PATH="$dir/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" CASADIPATH="$dir/lib" OPENBLAS_NUM_THREADS=1 "$@"
    fi
}
run_target "$work/parity" "$2"
run_target "$work/solver"
