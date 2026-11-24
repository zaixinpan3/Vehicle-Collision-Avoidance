#!/usr/bin/env bash
# Build the streaming controller using one coherent set of exported sources.
set -euo pipefail
if [[ $# != 3 ]]; then
    echo "Usage: $0 EXPORTED_SOURCES CASADI_DISTRIBUTION OUTPUT_DIRECTORY" >&2
    exit 2
fi
src=$(dirname "$(realpath "$0")")
generated=$(realpath "$1")
casadi=$(realpath "$2")
output=$(realpath -m "$3")
cmake -S "$src" -B "$generated/../native-build" -DCMAKE_BUILD_TYPE=Release \
    -DGENERATED_DIR="$generated" -DCASADI_ROOT="$casadi" -DCMAKE_INSTALL_PREFIX="$output"
cmake --build "$generated/../native-build" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
cmake --install "$generated/../native-build"
declare -A copied
copy_library() {
    local name=$1 dependency
    if [[ -n ${copied[$name]:-} ]]; then return; fi
    copied[$name]=1
    cp -L "$casadi/$name" "$output/lib/$name"
    while read -r dependency; do
        if [[ -f $casadi/$dependency ]]; then copy_library "$dependency"; fi
    done < <(readelf -d "$casadi/$name" | sed -n 's/.*(NEEDED).*\[\([^]]*\)\].*/\1/p')
}
copy_library libcasadi_nlpsol_ipopt.so
mkdir -p "$output/licenses"
cp -R "$casadi/include/licenses/." "$output/licenses/"
sed -n '1,/^ \*\/$/p' "$generated/target/nrmmTargetSample.c" > "$output/licenses/MathWorks-generated-code.txt"
cp "$src/PC_CONTROLLER.md" "$output/"
