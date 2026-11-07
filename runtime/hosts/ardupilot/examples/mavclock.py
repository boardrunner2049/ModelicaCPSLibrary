"""The autopilot's clock, for the mission checks.

Timeouts are in the autopilot's seconds (time_boot_ms of the messages it
sends), not wall-clock seconds, so a check means the same at any SITL speed,
including a vehicle model slower than real time.
"""
import time


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
