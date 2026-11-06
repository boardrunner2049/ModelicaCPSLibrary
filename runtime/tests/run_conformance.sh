#!/usr/bin/env bash
# Build the example vehicle FMUs, prepare descriptors and linkable shims, build
# the runtime with sanitizers and the four models linked in, and run the checker
# tests, the runtime conformance tests and the standalone host.
#
#   runtime/tests/run_conformance.sh [--rebuild-fmus]
#
# CPV_LEGACY_FMU=path/to/FastDyn_Copter.fmu adds the legacy-profile checks
# (an FMU without the version marker, e.g. FastDyn's out/fmi3/Copter).
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
out=$root/build/vehicle-interface
build=$root/build/runtime
mkdir -p "$out"
cd "$root"

declare -A source=([quad]=configs/quad.toml [hex]=configs/hex.toml
                   [rover]=Exports.FMU.Rover [plane]=Exports.FMU.FixedWing)
vehicles=(quad hex rover plane)
for vehicle in "${vehicles[@]}"; do
  if [[ ${1:-} == --rebuild-fmus || ! -f $out/$vehicle.fmu ]]; then
    python3 tools/vehicle_interface/build_vehicle_fmu.py "${source[$vehicle]}" "$out/$vehicle.fmu"
  fi
done
cvi() { (cd tools/vehicle_interface && python3 -m cp_vehicle_interface "$@"); }
for vehicle in "${vehicles[@]}"; do
  cvi prepare "$out/$vehicle.fmu" "$out/$vehicle" --shim "$vehicle" \
    --require pwm_actuation_v1 --require kinematic_state_v1 --require imu_v1
done
models=$(printf "$out/%s/shim;" "${vehicles[@]}")
rm -rf "$out/legacy"
if [[ -n ${CPV_LEGACY_FMU:-} ]]; then
  cvi prepare "$CPV_LEGACY_FMU" "$out/legacy" --profile fastdyn_legacy
fi

cmake -S runtime -B "$build" -DCPV_SANITIZE=ON -DCPV_TEST_VEHICLES="$out" \
  -DCPV_MODELS="${models%;}" >/dev/null
cmake --build "$build" -j >/dev/null
python3 -m unittest tools/vehicle_interface/tests/test_checker.py
ctest --test-dir "$build" --output-on-failure
for vehicle in quad hex; do
  "$build/cpv_hover_host" "$out/$vehicle/vehicle.json" 1.0 12 | tail -1
done
