"""The autopilot's clock, for the mission checks.

Timeouts are in the autopilot's seconds (time_boot_ms of the messages it
sends), not wall-clock seconds, so a check means the same at any SITL speed,
including a vehicle model slower than real time.
"""
from collections import namedtuple
import math
import time

from pymavlink import mavutil


class AutopilotClock:
    """Receives messages and keeps `now`, seconds since the autopilot booted.

    Raises TimeoutError when the autopilot's clock has not advanced for
    `stall` wall-clock seconds (SITL stopped or the link is gone).
    """

    def __init__(self, link, stall=120.0):
        self.link = link
        self.stall = stall
        self.now = 0.0
        self._advanced = time.monotonic()

    def recv(self):
        msg = self.link.recv_match(blocking=True, timeout=0.5)
        boot_ms = getattr(msg, "time_boot_ms", None)
        if boot_ms is not None and boot_ms / 1e3 > self.now:
            self.now = boot_ms / 1e3
            self._advanced = time.monotonic()
        if time.monotonic() - self._advanced > self.stall:
            raise TimeoutError(f"the autopilot's clock has not advanced for {self.stall:.0f} s")
        return msg


Position = namedtuple("Position", "x y z")


class Truth:
    """The vehicle's true position from SITL's SIM_STATE, in metres NED from
    where it was first seen (the vehicle at rest at home).

    The mission checks judge what the vehicle did, not what its estimator
    believes: with the model's own GNSS, barometer and compass the two differ
    by the sensor errors. `local` is None until SIM_STATE arrives.
    """

    EARTH_RADIUS = 6378137.0

    def __init__(self, link):
        self.local = None
        self._origin = None
        # SIM_STATE (108) at 20 Hz; SITL sends it only when asked.
        link.mav.command_long_send(link.target_system, link.target_component,
                                   mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   108, 50000, 0, 0, 0, 0, 0)

    def update(self, msg):
        if msg is None or msg.get_type() != "SIM_STATE":
            return
        lat = getattr(msg, "lat_int", 0) * 1e-7 or msg.lat * 1e-7
        lon = getattr(msg, "lon_int", 0) * 1e-7 or msg.lon * 1e-7
        if self._origin is None:
            self._origin = (lat, lon, msg.alt)
        lat0, lon0, alt0 = self._origin
        north = math.radians(lat - lat0) * self.EARTH_RADIUS
        east = math.radians(lon - lon0) * self.EARTH_RADIUS * math.cos(math.radians(lat0))
        self.local = (north, east, -(msg.alt - alt0))

    def position(self, local_position_ned):
        """Truth when known, else the estimator's LOCAL_POSITION_NED."""
        if self.local is not None:
            return Position(*self.local)
        msg = local_position_ned
        return Position(msg.x, msg.y, msg.z)
