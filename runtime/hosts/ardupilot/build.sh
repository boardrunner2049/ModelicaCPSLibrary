#!/usr/bin/env bash
# Build ArduPilot SITL with CP Models vehicles linked in.
#
#   runtime/hosts/ardupilot/build.sh [VEHICLE...]
#
# VEHICLE names a configs/VEHICLE.toml (default: quad hex rover plane rover_i2).
# Each becomes the SITL model `--model cpvehicle:VEHICLE`. WAF_TARGETS selects
# the autopilots (default: copter rover plane).
#
# Steps, all idempotent:
#   1. third_party/ardupilot at its pinned upstream commit (git submodule);
#   2. the three ArduPilot hooks (hooks.patch: model table entry, --cpvehicle
#      waf option, link flags) applied to that working tree, and this
#      directory's SIM_CPVehicle.{cpp,h} linked into libraries/SITL;
#   3. each configuration composed and exported to an FMU (rebuilt when the
#      configuration is newer), checked, and written as a linkable shim;
#   4. the vehicle runtime built with those models (build/runtime-release);
#   5. ArduPilot configured with --cpvehicle and built.
#
# Needs: the ArduPilot build Python packages (empy==3.3.4, future, pexpect,
# dronecan, pymavlink) on PATH, cargo, cmake, a C/C++ toolchain, the pinned
# Rumoca (tools/fmi). The submodule's working tree is modified by step 2; the
# pinned commit is not (.gitmodules: ignore = dirty).
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../../.." && pwd)
ap=$root/third_party/ardupilot
if (($#)); then vehicles=("$@"); else vehicles=(quad hex rover plane rover_i2); fi
read -r -a targets <<< "${WAF_TARGETS:-copter rover plane}"
cd "$root"

echo "== ArduPilot $(git submodule status third_party/ardupilot | awk '{print $1}' | tr -d '+-')"
git submodule update --init third_party/ardupilot
# The nested modules a SITL build uses (not ChibiOS and the other board-only ones).
git -C "$ap" submodule update --init --recursive modules/waf modules/mavlink modules/gtest \
  modules/DroneCAN
if ! git -C "$ap" apply --reverse --check "$here/hooks.patch" 2>/dev/null; then
  git -C "$ap" apply "$here/hooks.patch"
fi
for f in SIM_CPVehicle.cpp SIM_CPVehicle.h; do
  ln -sfn "$here/src/$f" "$ap/libraries/SITL/$f"
done

echo "== vehicles: ${vehicles[*]}"
out=$root/build/vehicle-interface
mkdir -p "$out"
models=()
for v in "${vehicles[@]}"; do
  config=configs/$v.toml
  [[ -f $config ]] || { echo "no $config" >&2; exit 2; }
  if [[ ! -f $out/$v.fmu || $config -nt $out/$v.fmu ]]; then
    python3 tools/vehicle_interface/build_vehicle_fmu.py "$config" "$out/$v.fmu"
  fi
  (cd tools/vehicle_interface && python3 -m cp_vehicle_interface prepare "$out/$v.fmu" "$out/$v" \
     --shim "$v" --require pwm_actuation_v1 --require kinematic_state_v1 --require imu_v1 | head -1)
  models+=("$out/$v/shim")
done

echo "== runtime"
cmake -S runtime -B build/runtime-release -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DCPV_MODELS="$(IFS=';'; echo "${models[*]}")" > /dev/null
cmake --build build/runtime-release -j > /dev/null

echo "== ArduPilot SITL: ${targets[*]}"
cd "$ap"
./waf configure --board sitl --cpvehicle="$root" > /dev/null
./waf "${targets[@]}"
echo "binaries: $ap/build/sitl/bin; run: $here/examples/view.sh NAME"
