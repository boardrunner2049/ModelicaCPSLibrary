#!/usr/bin/env python3
"""Fly an AUTO mission on ArduRover or ArduPlane SITL and check it.

usage: fly_auto.py rover|plane [--connect tcp:127.0.0.1:5760] [--home LAT,LON]

rover: a 20 m square back to home.
plane: runway takeoff north to 30 m, a 400 x 200 m circuit at 50 m, a
       straight-in approach from the south and an automatic landing at home.

Uploads the mission, waits for the EKF to use GPS, arms, flies it in AUTO
and checks every waypoint, the cross-track error on each leg, attitude
limits and the final position. Exits nonzero with the reason on the first
failed check.
"""
import argparse
import math
import sys

from pymavlink import mavutil

from mavclock import AutopilotClock

MAV = mavutil.mavlink
EARTH_RADIUS = 6378137.0

# (command, north m, east m, altitude m, param1); altitude relative to home.
MISSIONS = {
    "rover": [
        (MAV.MAV_CMD_NAV_WAYPOINT, 20, 0, 0, 0),
        (MAV.MAV_CMD_NAV_WAYPOINT, 20, 20, 0, 0),
        (MAV.MAV_CMD_NAV_WAYPOINT, 0, 20, 0, 0),
        (MAV.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0),
    ],
    "plane": [
        (MAV.MAV_CMD_NAV_TAKEOFF, 0, 0, 30, 12),  # param1: minimum pitch, deg
        (MAV.MAV_CMD_NAV_WAYPOINT, 250, 0, 50, 0),
        (MAV.MAV_CMD_NAV_WAYPOINT, 250, 200, 50, 0),
        (MAV.MAV_CMD_NAV_WAYPOINT, -350, 200, 50, 0),
        (MAV.MAV_CMD_NAV_WAYPOINT, -350, 0, 30, 0),
        (MAV.MAV_CMD_NAV_WAYPOINT, -150, 0, 12, 0),
        (MAV.MAV_CMD_NAV_LAND, 60, 0, 0, 0),
    ],
}
# Per vehicle: cross-track limit on legs [m], reach radius at the last item [m],
# attitude limit [deg].
LIMITS = {
    "rover": dict(cross_track=1.5, final=2.0, attitude=15.0),
    "plane": dict(cross_track=25.0, final=40.0, attitude=70.0),
}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("vehicle", choices=sorted(MISSIONS))
    p.add_argument("--connect", default="tcp:127.0.0.1:5760")
    p.add_argument("--home", default="40.414929,-86.932387")
    p.add_argument("--timeout", type=float, help="autopilot seconds", default=900.0)
    a = p.parse_args()
    lat0, lon0 = (float(x) for x in a.home.split(","))
    mission = MISSIONS[a.vehicle]
    limits = LIMITS[a.vehicle]

    m = mavutil.mavlink_connection(a.connect, autoreconnect=True)
    clock = AutopilotClock(m)
    deadline = a.timeout  # autopilot seconds since boot
    m.wait_heartbeat(timeout=60)
    print(f"heartbeat from system {m.target_system}")
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   MAV.MAV_DATA_STREAM_ALL, 10, 1)
    state = {"attitude": 0.0, "reached": set(), "track": []}

    def pump(seconds):
        end = clock.now + seconds
        while clock.now < end:
            msg = clock.recv()
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == "STATUSTEXT":
                print(f"  [fc] {msg.text}")
            elif kind == "ATTITUDE" and state.get("armed"):
                state["attitude"] = max(state["attitude"],
                                        math.degrees(max(abs(msg.roll), abs(msg.pitch))))
            elif kind == "LOCAL_POSITION_NED":
                state["local"] = msg
                if state.get("armed"):
                    state["track"].append((state.get("leg", 0), msg.x, msg.y, -msg.z))
            elif kind == "GLOBAL_POSITION_INT":
                state["global"] = msg
            elif kind == "VFR_HUD":
                state["hud"] = msg
            elif kind == "EKF_STATUS_REPORT":
                state["ekf_flags"] = msg.flags
            elif kind == "MISSION_CURRENT":
                state["leg"] = msg.seq
            elif kind == "MISSION_ITEM_REACHED":
                if msg.seq not in state["reached"]:
                    print(f"reached item {msg.seq}")
                state["reached"].add(msg.seq)
            elif kind == "HEARTBEAT" and msg.get_srcSystem() == m.target_system:
                state["armed"] = bool(msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)

    def fail(reason):
        print(f"FAILED: {reason}")
        sys.exit(1)

    def wait_until(predicate, what, timeout):
        end = min(clock.now + timeout, deadline)
        while clock.now < end:
            pump(0.5)
            if predicate():
                return
        fail(f"timed out waiting for {what}")

    def to_global(north, east):
        lat = lat0 + math.degrees(north / EARTH_RADIUS)
        lon = lon0 + math.degrees(east / (EARTH_RADIUS * math.cos(math.radians(lat0))))
        return int(round(lat * 1e7)), int(round(lon * 1e7))

    need = MAV.EKF_POS_HORIZ_ABS | MAV.EKF_VELOCITY_HORIZ | MAV.EKF_POS_VERT_ABS
    wait_until(lambda: state.get("ekf_flags", 0) & need == need, "EKF position from GPS", 180)
    print("EKF has a GPS position")
    # Upload: item 0 is home, then the mission.
    items = [(MAV.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0)] + mission
    m.mav.mission_count_send(m.target_system, m.target_component, len(items))
    sent = set()
    end = clock.now + 30
    while clock.now < end:
        msg = clock.recv()
        if msg is None or msg.get_type() not in ("MISSION_REQUEST", "MISSION_REQUEST_INT",
                                                 "MISSION_ACK"):
            continue
        if msg.get_type() == "MISSION_ACK":
            if msg.type != MAV.MAV_MISSION_ACCEPTED:
                fail(f"mission refused (MAV_MISSION_RESULT {msg.type})")
            break
        cmd, north, east, alt, p1 = items[msg.seq]
        lat, lon = to_global(north, east)
        m.mav.mission_item_int_send(m.target_system, m.target_component, msg.seq,
                                    MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd, 0, 1,
                                    p1, 0, 0, 0, lat, lon, alt)
        sent.add(msg.seq)
    else:
        fail("mission upload timed out")
    if len(sent) != len(items):
        fail(f"mission upload incomplete: {len(sent)} of {len(items)} items")
    print(f"mission uploaded: {len(mission)} items")

    m.mav.command_long_send(m.target_system, m.target_component, MAV.MAV_CMD_MISSION_START,
                            0, 0, 0, 0, 0, 0, 0, 0)
    m.set_mode("AUTO")
    end = clock.now + 180
    while not state.get("armed"):
        if clock.now > end:
            fail("could not arm within 180 s")
        m.mav.command_long_send(m.target_system, m.target_component,
                                MAV.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        pump(2)
    print("armed in AUTO")

    last = len(items) - 1
    if a.vehicle == "plane":
        # Landing ends on the ground, rolled out to a stop on the last leg.
        wait_until(lambda: state.get("leg") == last and "hud" in state
                   and state["hud"].groundspeed < 0.5 and -state["local"].z < 0.5,
                   "landing and rollout", deadline - clock.now)
        last_waypoint = last - 1
    else:
        wait_until(lambda: last in state["reached"], f"mission item {last}", deadline - clock.now)
        last_waypoint = last
    missing = [i for i in range(1, last_waypoint + 1) if i not in state["reached"]]
    if missing:
        fail(f"items not reached: {missing}")

    # Cross-track error on each leg between consecutive waypoints, after
    # the vehicle has had 1/4 of the leg to converge onto it.
    worst = 0.0
    for seq in range(2, len(items)):
        _, n0, e0, _, _ = items[seq - 1]
        _, n1, e1, _, _ = items[seq]
        length = math.hypot(n1 - n0, e1 - e0)
        if length < 1:
            continue
        for leg, x, y, _ in state["track"]:
            if leg != seq:
                continue
            along = ((x - n0) * (n1 - n0) + (y - e0) * (e1 - e0)) / length
            if along < 0.25 * length or along > length:
                continue
            cross = abs((x - n0) * (e1 - e0) - (y - e0) * (n1 - n0)) / length
            worst = max(worst, cross)
    print(f"worst cross-track error {worst:.2f} m, max roll/pitch {state['attitude']:.1f} deg")
    if worst > limits["cross_track"]:
        fail(f"cross-track error {worst:.2f} m > {limits['cross_track']} m")
    if state["attitude"] > limits["attitude"]:
        fail(f"attitude {state['attitude']:.1f} deg > {limits['attitude']} deg")

    _, n_end, e_end, _, _ = items[last]
    if a.vehicle == "plane":
        # Touchdown: the first sample on the landing leg within 0.3 m of the ground.
        touch = next(((x, y) for leg, x, y, alt in state["track"] if leg == last and alt < 0.3),
                     None)
        if touch is None:
            fail("no touchdown recorded")
        miss = math.hypot(touch[0] - n_end, touch[1] - e_end)
        loc = state["local"]
        print(f"touchdown at ({touch[0]:.1f}, {touch[1]:.1f}) m, {miss:.1f} m from the landing "
              f"point; stopped at ({loc.x:.1f}, {loc.y:.1f}) m, {abs(loc.y):.2f} m off the "
              f"centreline")
        if abs(loc.y) > 3.0:
            fail(f"rollout left the centreline by {abs(loc.y):.2f} m")
    else:
        pump(3)
        loc = state["local"]
        miss = math.hypot(loc.x - n_end, loc.y - e_end)
        print(f"stopped at ({loc.x:.2f}, {loc.y:.2f}) m, {miss:.2f} m from the last item")
    if miss > limits["final"]:
        fail(f"{miss:.2f} m from the last item (limit {limits['final']} m)")
    print("PASSED")


if __name__ == "__main__":
    try:
        main()
    except TimeoutError as e:
        print(f"FAILED: {e}")
        sys.exit(1)
