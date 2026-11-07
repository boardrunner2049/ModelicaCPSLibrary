#!/usr/bin/env python3
"""Fly a short guided mission on ArduCopter SITL and check it.

usage: fly_mission.py [--connect tcp:127.0.0.1:5760] [--alt 5] [--north 20]

Waits for the EKF to accept GPS, arms in GUIDED, takes off, holds, flies
north, comes back, lands, and checks altitude, position and disarm. Exits
nonzero with the reason on the first failed check.
"""
import argparse
import math
import sys

from pymavlink import mavutil

from mavclock import AutopilotClock


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--connect", default="tcp:127.0.0.1:5760")
    p.add_argument("--alt", type=float, default=5.0)
    p.add_argument("--north", type=float, default=20.0)
    p.add_argument("--timeout", type=float, help="autopilot seconds", default=600.0)
    a = p.parse_args()
    m = mavutil.mavlink_connection(a.connect, autoreconnect=True)
    clock = AutopilotClock(m)
    deadline = a.timeout  # autopilot seconds since boot
    m.wait_heartbeat(timeout=60)
    print(f"heartbeat from system {m.target_system}")
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    state = {"tilt": 0.0}

    def pump(seconds):
        end = clock.now + seconds
        last = None
        while clock.now < end:
            msg = clock.recv()
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == "STATUSTEXT":
                print(f"  [fc] {msg.text}")
            elif kind == "ATTITUDE" and state.get("armed"):
                state["tilt"] = max(state["tilt"], math.degrees(max(abs(msg.roll), abs(msg.pitch))))
            elif kind == "LOCAL_POSITION_NED":
                last = msg
                state["local"] = msg
            elif kind == "EKF_STATUS_REPORT":
                state["ekf_flags"] = msg.flags
            elif kind == "COMMAND_ACK":
                state.setdefault("acks", {})[msg.command] = msg.result
            elif kind == "HEARTBEAT" and msg.get_srcSystem() == m.target_system:
                state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return last

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

    # Wait until the EKF has an absolute horizontal position (from GPS).
    need = (mavutil.mavlink.EKF_POS_HORIZ_ABS | mavutil.mavlink.EKF_VELOCITY_HORIZ |
            mavutil.mavlink.EKF_POS_VERT_ABS)
    wait_until(lambda: state.get("ekf_flags", 0) & need == need, "EKF position from GPS", 180)
    print("EKF has a GPS position")
    # Arm in GUIDED once pre-arm checks pass.
    m.set_mode("GUIDED")
    end = clock.now + 180
    while not state.get("armed"):
        if clock.now > end:
            fail("could not arm within 180 s")
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        pump(2)
    print("armed")
    state.setdefault("acks", {}).pop(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, None)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, a.alt)
    wait_until(lambda: mavutil.mavlink.MAV_CMD_NAV_TAKEOFF in state["acks"], "takeoff ack", 10)
    if state["acks"][mavutil.mavlink.MAV_CMD_NAV_TAKEOFF] != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        fail(f"takeoff refused (MAV_RESULT {state['acks'][mavutil.mavlink.MAV_CMD_NAV_TAKEOFF]})")
    wait_until(lambda: "local" in state and -state["local"].z >= 0.95 * a.alt,
               f"altitude {a.alt} m", 60)
    print(f"reached {-state['local'].z:.2f} m")
    pump(5)
    loc = state["local"]
    if abs(-loc.z - a.alt) > 0.5:
        fail(f"hover altitude {-loc.z:.2f} m, expected {a.alt} m")
    print(f"hover: altitude {-loc.z:.2f} m, drift {math.hypot(loc.x, loc.y):.2f} m")

    def goto(north, east):
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000, north, east, -a.alt, 0, 0, 0, 0, 0, 0, 0, 0)

    goto(a.north, 0)
    wait_until(lambda: math.hypot(state["local"].x - a.north, state["local"].y) < 1.0,
               f"{a.north} m north", 90)
    print(f"north: ({state['local'].x:.2f}, {state['local'].y:.2f}) m")
    goto(0, 0)
    wait_until(lambda: math.hypot(state["local"].x, state["local"].y) < 1.0, "return", 90)
    print(f"back: ({state['local'].x:.2f}, {state['local'].y:.2f}) m")
    m.set_mode("LAND")
    wait_until(lambda: not state.get("armed"), "landing and disarm", 120)
    loc = state["local"]
    print(f"landed and disarmed at ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f}) m; max tilt "
          f"{state['tilt']:.1f} deg")
    if math.hypot(loc.x, loc.y) > 1.5:
        fail("landed more than 1.5 m from home")
    print("PASSED")


if __name__ == "__main__":
    try:
        main()
    except TimeoutError as e:
        print(f"FAILED: {e}")
        sys.exit(1)
