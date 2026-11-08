#!/usr/bin/env bash
# Generate a vehicle from its TOML and fly it in ArduPilot SITL.
#
#   runtime/hosts/ardupilot/run.sh VEHICLE [--mission] [--headless] [--speedup N] [--instance N] [--no-build]
#
# VEHICLE: a TOML file (a vehicle configuration, or a run file with [sitl] and
# [physics]; see runspec.py and examples/*.toml) or a configs/ name.
#
# Builds the vehicle (FMU, shim, runtime, the one autopilot it needs; an
# unchanged vehicle is not recompiled), starts SITL with it and opens MAVProxy
# with a map and console; the terminal is the MAV> prompt.
#   --mission   also fly the vehicle kind's mission check while you watch
#   --headless  no map: fly the mission check and exit with its result
#   --no-build  use the binaries as built
#   --instance N  SITL instance N (ports +10N), to run several vehicles at once
# MAVProxy: build/venv-gcs/bin/mavproxy.py, mavproxy.py on PATH, or MAVPROXY.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../../.." && pwd)
ap=$root/third_party/ardupilot
target=${1:?usage: run.sh VEHICLE [--mission] [--headless] [--speedup N] [--instance N] [--no-build]}
shift
mission= headless= build=1 speedup= instance=0
while (($#)); do
  case $1 in
    --mission) mission=1 ;;
    --headless) headless=1 mission=1 ;;
    --no-build) build= ;;
    --speedup) speedup=${2:?}; shift ;;
    --instance) instance=${2:?}; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done
eval "$(python3 "$here/runspec.py" "$target")"
speedup=${speedup:-$SPEEDUP}
port=$((5760 + 10 * instance)) gcs=$((14550 + 10 * instance)) check_port=$((14561 + 10 * instance))

if [[ -n $build ]]; then
  WAF_TARGETS=$WAF_TARGET "$here/build.sh" "$target" | grep -E '^==|conforms|finished' || {
    echo "build failed; run $here/build.sh $target to see why" >&2; exit 1; }
fi

# The vehicle's speed bounds SITL's: SITL waits for the model every frame. Measured
# with every PWM channel at 1500 us (motors running), slower than at rest.
rate=$("$root/build/runtime-release/cpv_realtime" "$NAME" 1200 1500 2>/dev/null | awk '{print $2}')
if awk -v r="$rate" -v s="$speedup" 'BEGIN { exit !(r < s) }'; then
  echo "WARNING: $NAME runs at ${rate}x real time, below --speedup $speedup: SITL runs at" \
       "most that fast. Sampled IMUs cost speed; an electromechanical IMU (kHz MEMS" \
       "dynamics) is far slower than real time." >&2
else
  echo "$NAME runs at ${rate}x real time"
fi

python=$root/build/venv-gcs/bin/python
[[ -x $python ]] || python=python3
case $KIND in
  multirotor) check=("$here/examples/fly_mission.py") ;;
  rover) check=("$here/examples/fly_auto.py" rover) ;;
  fixed_wing) check=("$here/examples/fly_auto.py" plane) ;;
esac

work=${RUN_WORK:-$HOME/.cache/cpvehicle-run/$NAME}
mkdir -p "$work" && cd "$work"
"$ap/build/sitl/bin/$AUTOPILOT" --model "cpvehicle:$NAME" --defaults "$PARAMETERS" \
  --home "$HOME_LOCATION" --speedup "$speedup" -I"$instance" -w > sitl.log 2>&1 &
sitl=$!
pids=($sitl)
trap 'kill "${pids[@]}" 2>/dev/null; wait 2>/dev/null' EXIT
sleep 2
kill -0 $sitl 2>/dev/null || { cat sitl.log; exit 1; }
grep -m1 'cpvehicle:' sitl.log
echo "SITL log: $work/sitl.log"

if [[ -n $headless ]]; then
  "$python" -u -W ignore "${check[@]}" --connect tcp:127.0.0.1:$port &
  pids+=($!)
  wait $!
  exit
fi
mavproxy=${MAVPROXY:-$root/build/venv-gcs/bin/mavproxy.py}
[[ -x $mavproxy ]] || mavproxy=$(command -v mavproxy.py) || {
  echo "MAVProxy not found; see $here/README.md" >&2; exit 2; }
if [[ -n $mission ]]; then
  "$python" -u -W ignore "${check[@]}" --connect udpin:127.0.0.1:$check_port > mission.log 2>&1 &
  pids+=($!)
  echo "mission check: $work/mission.log"
fi
"$mavproxy" --master tcp:127.0.0.1:$port --map --console --out udp:127.0.0.1:$gcs \
  --out udp:127.0.0.1:$check_port --state-basedir "$work" ${RUN_NONINTERACTIVE:+--non-interactive}
