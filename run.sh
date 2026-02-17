#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$SCRIPT_DIR/build}"
CONFIG="${CONFIG:-Release}"
RAYLIB_FETCHCONTENT="${RAYLIB_FETCHCONTENT:-ON}"
CMAKE_POLICY_VERSION_MINIMUM="${CMAKE_POLICY_VERSION_MINIMUM:-3.5}"

cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" \
  -DRAYLIB_FETCHCONTENT="$RAYLIB_FETCHCONTENT" \
  -DCMAKE_POLICY_VERSION_MINIMUM="$CMAKE_POLICY_VERSION_MINIMUM"
cmake --build "$BUILD_DIR" --config "$CONFIG"

CANDIDATES=(
  "$BUILD_DIR/voxel_dda_raylib"
  "$BUILD_DIR/voxel_dda_raylib.exe"
  "$BUILD_DIR/$CONFIG/voxel_dda_raylib"
  "$BUILD_DIR/$CONFIG/voxel_dda_raylib.exe"
  "$BUILD_DIR/Release/voxel_dda_raylib"
  "$BUILD_DIR/Release/voxel_dda_raylib.exe"
  "$BUILD_DIR/Debug/voxel_dda_raylib"
  "$BUILD_DIR/Debug/voxel_dda_raylib.exe"
  "$BUILD_DIR/RelWithDebInfo/voxel_dda_raylib"
  "$BUILD_DIR/RelWithDebInfo/voxel_dda_raylib.exe"
)

for exe in "${CANDIDATES[@]}"; do
  if [[ -f "$exe" ]]; then
    exec "$exe"
  fi
done

echo "error: could not find built executable 'voxel_dda_raylib' in '$BUILD_DIR'" >&2
exit 1
