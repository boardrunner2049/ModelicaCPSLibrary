#!/usr/bin/env bash
# Run ArduPilot SITL with a linked CP Models vehicle and open MAVProxy (map + console).
#
#   view.sh NAME [SPEEDUP]     NAME: quad | hex | rover | rover_i2 | plane
#
# MAVProxy also forwards MAVLink to udp:127.0.0.1:14561 (a mission script or a
# second GCS can listen there) and 14550 (QGroundControl's default).
# Closing MAVProxy (or Ctrl-C) stops SITL. Build first with ../build.sh.
# MAVPROXY overrides the ground station (default: build/venv-gcs/bin/mavproxy.py,
# else mavproxy.py on PATH).
set -euo pipefail
name=${1:?usage: view.sh NAME [SPEEDUP]}
speedup=${2:-1}
ex=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$ex/../../../.." && pwd)
ap=$root/third_party/ardupilot
mavproxy=${MAVPROXY:-$root/build/venv-gcs/bin/mavproxy.py}
[[ -x $mavproxy ]] || mavproxy=$(command -v mavproxy.py) || {
  echo "MAVProxy not found; see runtime/hosts/ardupilot/README.md" >&2; exit 2; }
case $name in
  quad|hex) bin=arducopter; parm=$ap/Tools/autotest/default_params/copter.parm,$ex/cpvehicle_quad.parm ;;
  rover*)   bin=ardurover;  parm=$ap/Tools/autotest/default_params/rover.parm,$ex/cpvehicle_rover.parm ;;
  plane*)   bin=arduplane;  parm=$ap/Tools/autotest/models/plane.parm,$ex/cpvehicle_plane.parm ;;
  *) echo "unknown vehicle $name" >&2; exit 2 ;;
esac
work=${VIEW_WORK:-$HOME/.cache/cpvehicle-view/$name}
mkdir -p "$work" && cd "$work"
"$ap/build/sitl/bin/$bin" --model "cpvehicle:$name" --defaults "$parm" \
  --home 40.414929,-86.932387,149,0 --speedup "$speedup" -I0 -w > sitl.log 2>&1 &
sitl=$!
trap 'kill $sitl 2>/dev/null; wait $sitl 2>/dev/null' EXIT
sleep 2
kill -0 $sitl || { cat sitl.log; exit 1; }
grep -m1 'cpvehicle:' sitl.log || true
"$mavproxy" --master tcp:127.0.0.1:5760 --map --console \
  --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14561 --state-basedir "$work" ${VIEW_NONINTERACTIVE:+--non-interactive}
