"""ROS-free checks: VirtualNm3Serial through the real Nm3.send_ping(), geometry, and LS convergence."""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))                                       # nm3_beacon_sim package
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "..", "..")))  # repo root

from nm3driver.nm3driver import Nm3                       # noqa: E402
from examples.lst_square_loc import method1_linear_ls     # noqa: E402
from nm3_beacon_sim.geometry import circle_intersections, horizontal_radius  # noqa: E402
from nm3_beacon_sim.virtual_modem import VirtualNm3Serial  # noqa: E402

BEACON = (20.0, -15.0, -30.0)
Z_S, C = -2.0, 1500.0


def _modem(pose, sigma=0.0, seed=0, **kw):
    s = VirtualNm3Serial(lambda: pose, BEACON, Z_S, sigma, C, np.random.default_rng(seed), **kw)
    return s, Nm3(input_stream=s, output_stream=s)


class FakeClock:
    """Simulated time that advances a fixed step every time the modem looks at the clock."""
    def __init__(self, dt=1e-5):
        self.t, self.dt = 0.0, dt

    def __call__(self):
        self.t += self.dt
        return self.t


def _dist(p, b=BEACON):
    return float(np.linalg.norm(np.array([p[0], p[1], Z_S]) - np.asarray(b)))


def test_send_ping_returns_true_range_without_noise():
    for pose in [(0.0, 0.0), (35.0, 10.0), (-40.0, -60.0)]:
        s, nm3 = _modem(pose, propagation_delay=False)
        tof = nm3.send_ping(2)
        true = np.linalg.norm(np.array([*pose, Z_S]) - BEACON)
        assert tof > 0
        assert abs(tof * C - true) < 0.05                 # 31.25 us tick = 4.7 cm
        assert abs(s.last_true_range - true) < 1e-9


def test_no_pose_times_out():
    s = VirtualNm3Serial(lambda: None, BEACON, Z_S, 0.0, C, np.random.default_rng(0))
    assert Nm3(s, s).send_ping(2, timeout=0.2) == -1


def test_noise_std_is_about_half_metre():
    pose = [0.0, 0.0]
    s, nm3 = _modem(pose, sigma=0.5, seed=1, propagation_delay=False)
    true = np.linalg.norm(np.array([0.0, 0.0, Z_S]) - BEACON)
    err = np.array([nm3.send_ping(2) * C - true for _ in range(2000)])
    assert abs(err.mean()) < 0.05 and abs(err.std() - 0.5) < 0.05


def test_two_circle_intersection_contains_beacon():
    dz2 = (Z_S - BEACON[2]) ** 2
    p1, p2 = (0.0, 0.0), (30.0, 5.0)
    r = [horizontal_radius(np.linalg.norm(np.array([*p, Z_S]) - BEACON), dz2) for p in (p1, p2)]
    pts = circle_intersections(p1, r[0], p2, r[1])
    assert len(pts) == 2
    assert min(np.hypot(x - BEACON[0], y - BEACON[1]) for x, y in pts) < 1e-6


def test_ls_through_driver_converges():
    pose = [0.0, 0.0]
    s, nm3 = _modem(pose, sigma=0.5, seed=3)
    P, rho = [], []
    for a in np.linspace(0, 2 * np.pi, 40, endpoint=False):     # circle of radius 30 around (10, -5)
        pose[0], pose[1] = 10 + 30 * np.cos(a), -5 + 30 * np.sin(a)
        P.append(tuple(pose))
        rho.append(nm3.send_ping(2) * C)
    res = method1_linear_ls(np.array(P), np.array(rho), Z_S)
    assert res["ok"]
    e = res["x"] - BEACON
    assert np.hypot(e[0], e[1]) < 1.0 and abs(e[2]) < 3.0


# ---------------------------------------------------------------- acoustic propagation delay
def test_reply_is_held_for_round_trip_time():
    beacon = (150.0, 0.0, -30.0)                           # ~151 m -> ~0.2 s round trip
    s = VirtualNm3Serial(lambda: (0.0, 0.0), beacon, Z_S, 0.0, C, np.random.default_rng(0))
    t0 = time.monotonic()
    tof = Nm3(s, s).send_ping(2)
    elapsed = time.monotonic() - t0
    d = _dist((0.0, 0.0), beacon)
    assert abs(tof * C - d) < 0.05
    assert 2 * d / C - 0.005 < elapsed < 2 * d / C + 0.1
    assert abs(s.last_rtt - 2 * d / C) < 1e-3


def test_moving_vessel_measures_mean_of_both_legs():
    beacon, v = (1500.0, 0.0, -30.0), np.array([0.0, 10.0])   # ~2 s round trip at 10 m/s -> 20 m moved
    clk = FakeClock()
    pose_fn = lambda: tuple(v * clk.t)                         # noqa: E731
    s = VirtualNm3Serial(pose_fn, beacon, Z_S, 0.0, C, np.random.default_rng(0), clock=clk)
    tof = Nm3(s, s).send_ping(2, timeout=5.0)
    rho = tof * C
    p_send, p_recv = s.last_pose, s.last_recv_pose
    assert np.hypot(*np.subtract(p_recv, p_send)) > 15         # vessel really moved during the ping
    assert abs(rho - (_dist(p_send, beacon) + _dist(p_recv, beacon)) / 2) < 0.05
    mid = ((p_send[0] + p_recv[0]) / 2, (p_send[1] + p_recv[1]) / 2)
    assert abs(rho - _dist(mid, beacon)) < 0.1                 # midpoint is the right position


def test_sound_speed_mismatch_scales_range():
    s, nm3 = _modem((0.0, 0.0), c_true=1450.0, clock=FakeClock())
    assert abs(nm3.send_ping(2) * C - _dist((0.0, 0.0)) * C / 1450.0) < 0.05


def test_beacon_turnaround_adds_constant_bias():
    s, nm3 = _modem((0.0, 0.0), turnaround=0.010, clock=FakeClock())
    assert abs(nm3.send_ping(2) * C - (_dist((0.0, 0.0)) + C * 0.010 / 2)) < 0.05   # +7.5 m


def test_out_of_range_times_out():
    beacon = (4000.0, 0.0, -30.0)                            # 5.3 s round trip > 4 s modem timeout
    s = VirtualNm3Serial(lambda: (0.0, 0.0), beacon, Z_S, 0.0, C, np.random.default_rng(0),
                         clock=FakeClock(dt=1e-4))
    assert Nm3(s, s).send_ping(2, timeout=5.0) == -1
