"""Virtual NM3 serial port: stands in for pyserial so Nm3.send_ping() runs unchanged in simulation.

Protocol emulated (see nm3driver.Nm3.send_ping):
    host  -> '$Pnnn'
    modem -> '$Pnnn\\r\\n'              command acknowledged (immediately)
    modem -> '#RnnnTttttt\\r\\n'        time of flight, ttttt in 31.25 us ticks (driver: tof = ticks * 31.25e-6)
    modem -> '#TO\\r\\n'                no reply within NM3_RANGE_TIMEOUT

Acoustic propagation (propagation_delay=True, real time, live vessel pose):
    t_send                         ping leaves the vessel transducer at p_send = (x, y, z_s)
    t_hit  = t_send + d1 / c_true  reaches the beacon,          d1 = |p_send - b|
    t_emit = t_hit + turnaround    beacon replies (turnaround = 0 per the Method 1 assumption)
    t_recv = t_emit + d2 / c_true  reply reaches the vessel,    d2 = |p(t_recv) - b|
The vessel keeps moving during the ping, so d2 uses its pose at reception. The '#R' reply is
withheld until t_recv. The modem reports half the round trip (or the full round trip when
tof_scale = 0.5), which the host converts with c_modem:
    rho_reported = c_modem * (t_recv - t_send) / 2 + N(0, sigma)
                 = (c_modem / c_true) * ((d1 + d2) / 2) + c_modem * turnaround / 2 + noise
So a sound-speed mismatch scales the range and any uncompensated turnaround adds a constant bias.

propagation_delay=False keeps the old instantaneous behaviour (d1 = d2, reply queued at once).
"""
import re
import threading
import time

import numpy as np

TICK_S = 31.25e-6
NM3_RANGE_TIMEOUT = 4.0                  # modem gives up and sends '#TO' (driver comment: "after 4 seconds")
_PING_RE = re.compile(rb"\$P(\d{3})")


class VirtualNm3Serial:
    def __init__(self, pose_fn, beacon, z_s, sigma, c, rng, *, tof_scale=1.0, c_true=None,
                 turnaround=0.0, propagation_delay=True, clock=time.monotonic):
        self._pose_fn = pose_fn                    # () -> (x, y) of the agent now, or None
        self._beacon = np.asarray(beacon, float)
        self._z_s = float(z_s)
        self._sigma = float(sigma)
        self._c_modem = float(c)                   # speed of sound the host/modem assumes
        self._tof_scale = float(tof_scale)
        self._c_true = float(c_true) if c_true else self._c_modem   # speed the sound actually travels at
        self._turnaround = float(turnaround)
        self._delay = bool(propagation_delay)
        self._rng = rng
        self._clock = clock
        self._out = bytearray()
        self._pending = None                       # in-flight ping
        self._lock = threading.Lock()
        # diagnostics of the last completed ping
        self.last_true_range = float("nan")        # (d1 + d2) / 2: noise-free geometric mean range
        self.last_d1 = self.last_d2 = float("nan")
        self.last_pose = self.last_recv_pose = None
        self.last_rtt = float("nan")               # simulated acoustic round-trip time [s]

    def _dist(self, pose):
        return float(np.linalg.norm(np.array([pose[0], pose[1], self._z_s]) - self._beacon))

    def _reply(self, addr, rtt):
        """'#R' string for a measured round-trip time (rtt), with range noise."""
        rho = self._c_modem * rtt / 2 + self._sigma * self._rng.standard_normal()
        tof = max(rho, 0.0) / (self._c_modem * self._tof_scale)
        ticks = int(min(max(round(tof / TICK_S), 0), 99999))
        return b"#R%03dT%05d\r\n" % (addr, ticks)

    # pyserial-like API used by Nm3 -------------------------------------------------
    def write(self, data: bytes) -> int:
        m = _PING_RE.fullmatch(bytes(data))
        if m is None:
            return len(data)                       # other commands: accepted, no reply
        addr = int(m.group(1))
        pose = self._pose_fn()
        with self._lock:
            self._out += b"$P%03d\r\n" % addr
            if pose is None:
                self._out += b"#TO\r\n"
                self._pending = None
                return len(data)
            d1 = self._dist(pose)
            if not self._delay:                    # instantaneous: vessel assumed static
                rtt = 2 * d1 / self._c_true + self._turnaround
                self._out += self._reply(addr, rtt)
                self._record(d1, d1, pose, pose, rtt)
                self._pending = None
            else:
                t0 = self._clock()
                self._pending = dict(addr=addr, t_send=t0, d1=d1, pose=pose,
                                     t_emit=t0 + d1 / self._c_true + self._turnaround)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            self._advance()
            out, self._out = bytes(self._out), bytearray()
        return out

    def close(self):
        pass

    # ------------------------------------------------------------------ propagation
    def _advance(self):
        """Deliver the in-flight reply once the return wavefront has reached the vessel."""
        p = self._pending
        if p is None:
            return
        now = self._clock()
        if now - p["t_send"] > NM3_RANGE_TIMEOUT:
            self._out += b"#TO\r\n"
            self._pending = None
            return
        if now < p["t_emit"]:
            return
        pose = self._pose_fn()
        if pose is None:
            return
        d2 = self._dist(pose)
        t_recv = p["t_emit"] + d2 / self._c_true   # exact arrival for the vessel's current position
        if now < t_recv:
            return
        rtt = t_recv - p["t_send"]
        self._out += self._reply(p["addr"], rtt)
        self._record(p["d1"], d2, p["pose"], pose, rtt)
        self._pending = None

    def _record(self, d1, d2, pose_send, pose_recv, rtt):
        self.last_d1, self.last_d2 = d1, d2
        self.last_true_range = (d1 + d2) / 2
        self.last_pose = (float(pose_send[0]), float(pose_send[1]))
        self.last_recv_pose = (float(pose_recv[0]), float(pose_recv[1]))
        self.last_rtt = rtt
