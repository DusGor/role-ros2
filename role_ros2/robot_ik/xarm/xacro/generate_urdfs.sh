#!/usr/bin/env bash
# Regenerate the committed flat URDFs (../uf850_allegro_*.urdf) from the xacro
# sources in this folder. Requires xarm_description and allegro_hand_v4_description
# to be built and sourced (xacro resolves $(find ...) through the ament index).
#
#   ./generate_urdfs.sh          regenerate ../uf850_allegro_*.urdf
#   ./generate_urdfs.sh --check  fail if the committed URDFs are out of date
set -euo pipefail
cd "$(dirname "$0")"
models=(uf850_allegro_left uf850_allegro_right uf850_allegro_bimanual)

if [[ "${1:-}" == "--check" ]]; then
    tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
    status=0
    for m in "${models[@]}"; do
        xacro "$m.urdf.xacro" -o "$tmp/$m.urdf"
        diff -q "$tmp/$m.urdf" "../$m.urdf" >/dev/null || { echo "OUT OF DATE: ../$m.urdf"; status=1; }
    done
    exit $status
fi

for m in "${models[@]}"; do
    xacro "$m.urdf.xacro" -o "../$m.urdf"
    echo "generated ../$m.urdf"
done
