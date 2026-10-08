"""Load and validate the JSON experiment config; build the modem stream (sim or real serial)."""
import copy
import json
import os
import sys

DEFAULTS = {
    "paths":     {"nm3_repo": None},          # repo root holding nm3driver/ and examples/; None -> auto
    "modem":     {"mode": "sim", "serial_port": "/dev/ttyUSB0", "baud": 9600,
                  "beacon_address": 2, "timeout": 5.0, "speed_of_sound": 1500.0, "tof_scale": 1.0},
    "ping":      {"interval": 2.0},
    "geometry":  {"z_s": -2.0, "depth_prior": -30.0},
    "beacon":    {"true_xyz": None},
    "sim":       {"sigma_rho": 0.5, "seed": 7, "use_gazebo": True,
                  "propagation_delay": True,        # hold the reply for the real acoustic travel time
                  "true_speed_of_sound": None,      # speed sound actually travels at; None = modem.speed_of_sound
                  "beacon_turnaround": 0.0},        # uncompensated beacon reply delay [s]
    "pose":      {"topic": "/odom", "type": "odometry", "gps_datum": None},
    "estimator": {"weighted": False, "min_pings": 3,
                  "position": "midpoint"},          # ping position used in LS: send | receive | midpoint
    "viz":       {"frame": "odom", "max_circles": 50, "rviz": True},
    "logging":   {"enabled": True, "dir": "~/nm3_logs", "storage": "sqlite3",
                  "topics": ["/odom", "/cmd_vel", "/tf", "/tf_static", "/nm3/ping",
                             "/nm3/range", "/nm3/estimate", "/nm3/markers"]},
}


class ConfigError(ValueError):
    pass


def _merge(base, over, path=""):
    for k, v in over.items():
        if k not in base:
            raise ConfigError(f"unknown config key '{path}{k}'")
        if isinstance(base[k], dict) and isinstance(v, dict):
            _merge(base[k], v, f"{path}{k}.")
        else:
            base[k] = v
    return base


def load_config(path: str) -> dict:
    """Read the JSON file, fill in defaults for missing keys, and validate."""
    path = os.path.expanduser(path)
    try:
        with open(path) as f:
            user = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        raise ConfigError(f"cannot read config '{path}': {e}") from e
    cfg = _merge(copy.deepcopy(DEFAULTS), user)

    m = cfg["modem"]
    if m["mode"] not in ("sim", "serial"):
        raise ConfigError("modem.mode must be 'sim' or 'serial'")
    if not 0 <= int(m["beacon_address"]) <= 255:
        raise ConfigError("modem.beacon_address must be 0-255")
    xyz = cfg["beacon"]["true_xyz"]
    if xyz is not None and len(xyz) != 3:
        raise ConfigError("beacon.true_xyz must be [x, y, z] or null")
    if m["mode"] == "sim" and xyz is None:
        raise ConfigError("modem.mode='sim' needs beacon.true_xyz")
    if cfg["pose"]["type"] not in ("odometry", "navsatfix"):
        raise ConfigError("pose.type must be 'odometry' or 'navsatfix'")
    if cfg["estimator"]["position"] not in ("send", "receive", "midpoint"):
        raise ConfigError("estimator.position must be 'send', 'receive' or 'midpoint'")
    if cfg["sim"]["beacon_turnaround"] < 0:
        raise ConfigError("sim.beacon_turnaround must be >= 0")
    if cfg["estimator"]["min_pings"] < 3:
        raise ConfigError("estimator.min_pings must be >= 3 (unknowns x, y, gamma)")
    cfg["_path"] = os.path.abspath(path)
    return cfg


def repo_root(cfg: dict) -> str:
    """nm3-python-simulator root: from config, else derived from this file (ros2_ws/src/<pkg>/<pkg>/)."""
    if cfg["paths"]["nm3_repo"]:
        return os.path.abspath(os.path.expanduser(cfg["paths"]["nm3_repo"]))
    here = os.path.realpath(__file__)
    for _ in range(5):
        here = os.path.dirname(here)
    return here


def add_repo_to_path(cfg: dict) -> str:
    root = repo_root(cfg)
    if not os.path.isfile(os.path.join(root, "nm3driver", "nm3driver.py")):
        raise ConfigError(f"nm3driver not found under '{root}'; set paths.nm3_repo in the config")
    if root not in sys.path:
        sys.path.insert(0, root)
    return root


def make_modem_stream(cfg: dict, pose_fn):
    """Stream object handed to Nm3 as both input and output.

    sim    -> VirtualNm3Serial (replies computed from geometry + noise)
    serial -> pyserial port connected to the vessel's NM3 (same settings as ranging_example.py)
    """
    m = cfg["modem"]
    if m["mode"] == "sim":
        import numpy as np
        from .virtual_modem import VirtualNm3Serial
        s = cfg["sim"]
        return VirtualNm3Serial(pose_fn=pose_fn, beacon=cfg["beacon"]["true_xyz"],
                                z_s=cfg["geometry"]["z_s"], sigma=s["sigma_rho"],
                                c=m["speed_of_sound"], rng=np.random.default_rng(s["seed"]),
                                tof_scale=m["tof_scale"], c_true=s["true_speed_of_sound"],
                                turnaround=s["beacon_turnaround"],
                                propagation_delay=s["propagation_delay"])
    import serial
    return serial.Serial(m["serial_port"], m["baud"], 8, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.1)
