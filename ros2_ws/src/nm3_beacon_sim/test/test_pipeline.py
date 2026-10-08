"""ROS-free checks: VirtualNm3Serial through the real Nm3.send_ping(), geometry, and LS convergence."""
import os
import sys

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


def _modem(pose, sigma=0.0, seed=0):
    s = VirtualNm3Serial(lambda: pose, BEACON, Z_S, sigma, C, np.random.default_rng(seed))
    return s, Nm3(input_stream=s, output_stream=s)


def test_send_ping_returns_true_range_without_noise():
    for pose in [(0.0, 0.0), (35.0, 10.0), (-40.0, -60.0)]:
        s, nm3 = _modem(pose)
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
    s, nm3 = _modem(pose, sigma=0.5, seed=1)
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
