"""Virtual NM3 serial port: stands in for pyserial so Nm3.send_ping() runs unchanged in simulation.

Protocol emulated (see nm3driver.Nm3.send_ping):
    host  -> '$Pnnn'
    modem -> '$Pnnn\\r\\n'              command acknowledged
    modem -> '#RnnnTttttt\\r\\n'        time of flight, ttttt in 31.25 us ticks (driver: tof = ticks * 31.25e-6)

Assumptions (same as lst_square_loc Method 1): transducer at fixed depth z_s, no beacon
turnaround delay, Gaussian range noise sigma.
"""
import re
import threading

import numpy as np

TICK_S = 31.25e-6
_PING_RE = re.compile(rb"\$P(\d{3})")


class VirtualNm3Serial:
    def __init__(self, pose_fn, beacon, z_s, sigma, c, rng):
        self._pose_fn = pose_fn                    # () -> (x, y) of the agent, or None
        self._beacon = np.asarray(beacon, float)
        self._z_s = float(z_s)
        self._sigma = float(sigma)
        self._c = float(c)
        self._rng = rng
        self._out = bytearray()
        self._lock = threading.Lock()
        self.last_true_range = float("nan")
        self.last_pose = None

    # pyserial-like API used by Nm3 -------------------------------------------------
    def write(self, data: bytes) -> int:
        m = _PING_RE.fullmatch(bytes(data))
        if m is None:
            return len(data)                       # other commands: accepted, no reply
        addr = int(m.group(1))
        reply = b"$P%03d\r\n" % addr
        pose = self._pose_fn()
        if pose is None:
            reply += b"#TO\r\n"
        else:
            p = np.array([pose[0], pose[1], self._z_s])
            rho_true = float(np.linalg.norm(p - self._beacon))
            rho_meas = max(rho_true + self._sigma * self._rng.standard_normal(), 0.0)
            ticks = int(min(max(round(rho_meas / self._c / TICK_S), 0), 99999))
            reply += b"#R%03dT%05d\r\n" % (addr, ticks)
            self.last_true_range, self.last_pose = rho_true, (float(pose[0]), float(pose[1]))
        with self._lock:
            self._out += reply
        return len(data)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            out, self._out = bytes(self._out), bytearray()
        return out

    def close(self):
        pass
