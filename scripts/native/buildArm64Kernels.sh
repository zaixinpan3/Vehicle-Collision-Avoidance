#!/usr/bin/env bash
# Build from the C sources emitted by exportArm64Sources.
set -euo pipefail
if [[ $# != 2 ]]; then
    echo "Usage: $0 GENERATED_DIRECTORY OUTPUT_DIRECTORY" >&2
    exit 2
fi
: "${CASADI_ARM64_DIR:?Set CASADI_ARM64_DIR to the target CasADi distribution containing include/ and libcasadi.so}"
casadi=$(realpath "$CASADI_ARM64_DIR")
generated=$(realpath "$1")
output=$(realpath -m "$2")
cc=${CC:-aarch64-linux-gnu-gcc}
mkdir -p "$output/lib" "$output/include"
is_arm64() {
    readelf -h "$1" | awk '/Machine:/ { ok=($2=="AArch64") } END { exit !ok }'
}
is_arm64 "$casadi/libcasadi.so" || { echo "Expected a target CasADi distribution" >&2; exit 1; }
build() {
    local name=$1 directory=$2
    shift 2
    local headers=("$directory"/*.h) sources=("$directory"/*.c)
    "$cc" -O3 -DNDEBUG -mcpu="${PCBF_TARGET_CPU:-cortex-a76}" -ffp-contract=off -fPIC -shared -Wl,-Bsymbolic \
        -Wl,-z,defs -Wl,-soname,"lib$name.so" "${sources[@]}" "$@" -lm -o "$output/lib/lib$name.so"
    if ! is_arm64 "$output/lib/lib$name.so"; then
        echo "Expected an AArch64 compiler for $name" >&2
        exit 1
    fi
    mkdir -p "$output/include/$name"
    cp "${headers[@]}" "$output/include/$name/"
}
build pcbf_prediction "$generated/generated"
build pcbf_geometry "$generated/geometry"
build nrmm_interval "$generated/nrmm"
src=$(dirname "$(realpath "$0")")
build nrmm_target "$generated/target" -I"$generated/target" "$src/nrmm_track.c"
cp "$src/nrmm_track.h" "$output/include/"
cp "$generated/generated/configuration.mat" "$output/"
cxx=${CXX:-aarch64-linux-gnu-g++}
"$cxx" -std=c++17 -O3 -mcpu=cortex-a76 -ffp-contract=off -DNDEBUG -fPIC -shared \
    -I"$CASADI_ARM64_DIR/include" "$src/pcbf_ipopt.cpp" -L"$CASADI_ARM64_DIR" -lcasadi \
    -Wl,-z,defs -Wl,-rpath,'$ORIGIN' -o "$output/lib/libpcbf_ipopt.so"
is_arm64 "$output/lib/libpcbf_ipopt.so"
cp "$src/pcbf_ipopt.h" "$output/include/"

# Copy the solver plugin and only its target-distribution dependencies.
# System libraries are supplied by the target OS.
declare -A bundled
copy_runtime() {
    local name=$1 dependency
    if [[ -n ${bundled[$name]:-} ]]; then return; fi
    bundled[$name]=1
    is_arm64 "$casadi/$name"
    cp -L "$casadi/$name" "$output/lib/$name"
    while IFS= read -r dependency; do
        if [[ -e "$casadi/$dependency" ]]; then copy_runtime "$dependency"; fi
    done < <(readelf -d "$casadi/$name" | sed -n 's/.*(NEEDED).*\[\([^]]*\)\].*/\1/p')
}
copy_runtime libcasadi_nlpsol_ipopt.so
if [[ -d "$casadi/include/licenses" ]]; then
    mkdir -p "$output/licenses"
    cp -R "$casadi/include/licenses/." "$output/licenses/"
fi
