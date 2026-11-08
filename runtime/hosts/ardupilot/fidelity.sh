#!/usr/bin/env bash
# Fly one airframe with one part at each fidelity level in ArduPilot SITL.
#
#   runtime/hosts/ardupilot/fidelity.sh PART KIND [LEVEL...]
#
#   imu         quad | rover | plane   levels 0-3 (I0 ideal .. I3 electromechanical)
#   propulsion  quad                   levels 0-4 (P0 first-order speed .. P4 switching
#                                                  three-phase PM motor)
#
# The airframe is configs/KIND.toml; only PART differs (fidelity.py). The run
# files go to build/fidelity/PART_KIND_LEVEL.toml. Every level flies the vehicle
# kind's mission check headless, all levels at once (SITL instances 1..N), each
# at min(SPEEDUP, the model's real-time rate). Slow levels fly at their own
# rate: I3 takes 15-45 minutes.
#   SPEEDUP=N  (default 2)
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../../.." && pwd)
part=${1:?usage: fidelity.sh PART KIND [LEVEL...]}
kind=${2:?usage: fidelity.sh PART KIND [LEVEL...]}
shift 2
levels=("$@")
if ((${#levels[@]} == 0)); then
  case $part in imu) levels=(0 1 2 3) ;; propulsion) levels=(0 1 2 3 4) ;; *) levels=(0) ;; esac
fi
prefix=${part:0:1}
out=$root/build/fidelity
mkdir -p "$out"

files=()
for n in "${levels[@]}"; do
  f=$out/${part}_${kind}_$prefix$n.toml
  python3 "$here/fidelity.py" "$part" "$kind" "$n" "$f"
  files+=("$f")
done

echo "== building ${files[*]##*/}"
"$here/build.sh" "${files[@]}" | grep -E '^== (linked|ArduPilot SITL)|conforms' ||
  { echo "build failed; run $here/build.sh ${files[*]}" >&2; exit 1; }

speedup=${SPEEDUP:-2}
pids=()
for i in "${!files[@]}"; do
  name=$(basename "${files[$i]}" .toml)
  rate=$("$root/build/runtime-release/cpv_realtime" "$name" 1200 1500 2>/dev/null | awk '{print $2}')
  s=$(awk -v r="$rate" -v s="$speedup" 'BEGIN { print (r < s) ? 1 : s }')
  echo "$name: ${rate}x real time, flying at --speedup $s"
  "$here/run.sh" "${files[$i]}" --headless --no-build --speedup "$s" --instance $((i + 1)) \
    > "$out/$name.log" 2>&1 &
  pids+=($!)
done
trap 'kill "${pids[@]}" 2>/dev/null' EXIT

status=0
for i in "${!files[@]}"; do
  name=$(basename "${files[$i]}" .toml)
  wait "${pids[$i]}" && result=PASSED || { result=FAILED; status=1; }
  echo "== $name: $result (log $out/$name.log)"
  grep -E 'hover:|landed|worst|stopped|touchdown|FAILED' "$out/$name.log" | sed 's/^/   /'
done
trap - EXIT
exit $status
