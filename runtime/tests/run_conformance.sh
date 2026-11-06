#!/usr/bin/env bash
# Build the example vehicle FMUs from their configs/*.toml, prepare descriptors
# and linkable shims, build the runtime with sanitizers and every model linked
# in, and run the composer and checker tests, the runtime conformance tests and
# the standalone host. A configuration newer than its FMU rebuilds it;
# --rebuild-fmus rebuilds all (needed after Modelica source changes).
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

# Every vehicle is a fire-compose configuration; rover_i2 is rover with an I2 IMU.
vehicles=(quad hex rover plane rover_i2)
for vehicle in "${vehicles[@]}"; do
  config=configs/$vehicle.toml
  if [[ ${1:-} == --rebuild-fmus || ! -f $out/$vehicle.fmu || $config -nt $out/$vehicle.fmu ]]; then
    python3 tools/vehicle_interface/build_vehicle_fmu.py "$config" "$out/$vehicle.fmu"
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
cargo test -q --locked --offline --manifest-path tools/fire-compose/Cargo.toml
ctest --test-dir "$build" --output-on-failure
for vehicle in quad hex; do
  "$build/cpv_hover_host" "$out/$vehicle/vehicle.json" 1.0 12 | tail -1
done
