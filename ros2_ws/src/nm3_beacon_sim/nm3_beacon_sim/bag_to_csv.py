"""Export /nm3/ping from a rosbag2 recording to CSV, optionally re-solving with method1_linear_ls.

Usage:
    ros2 run nm3_beacon_sim bag_to_csv <run_dir_or_bag_dir> [-o pings.csv] [--solve]
"""
import argparse
import csv
import json
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray

from .config import add_repo_to_path, load_config
from .pinger_node import PING_FIELDS


def _bag_dir(path):
    path = os.path.expanduser(path)
    return os.path.join(path, "bag") if os.path.isdir(os.path.join(path, "bag")) else path


def read_pings(bag_dir, storage=None):
    with open(os.path.join(bag_dir, "metadata.yaml")) as f:
        meta = f.read()
    storage = storage or ("mcap" if "storage_identifier: mcap" in meta else "sqlite3")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/nm3/ping"]))
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        rows.append(list(deserialize_message(data, Float64MultiArray).data))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", help="run folder (contains bag/ and config.json) or the bag folder itself")
    ap.add_argument("-o", "--output", help="CSV path (default: <run_dir>/pings.csv)")
    ap.add_argument("--solve", action="store_true", help="re-run method1_linear_ls on the successful pings")
    a = ap.parse_args()

    bag = _bag_dir(a.run_dir)
    rows = read_pings(bag)
    out = a.output or os.path.join(os.path.dirname(bag) if bag.endswith("bag") else bag, "pings.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(PING_FIELDS)
        w.writerows(rows)
    print(f"{len(rows)} pings -> {out}")

    if a.solve:
        cfg_path = os.path.join(os.path.dirname(bag), "config.json")
        cfg = load_config(cfg_path)
        add_repo_to_path(cfg)
        from examples.lst_square_loc import method1_linear_ls
        width = min(len(r) for r in rows) if rows else 0
        R = np.array([r[:width] for r in rows if r[PING_FIELDS.index("ok")] > 0.5])
        if len(R) < 3:
            raise SystemExit(f"--solve needs at least 3 successful pings, bag has {len(R)}")
        i = PING_FIELDS.index
        # position given to LS live (x_used, y_used); bags recorded before it existed only have x, y
        xy = [i("x_used"), i("y_used")] if R.shape[1] > i("y_used") else [i("x"), i("y")]
        res = method1_linear_ls(R[:, xy], R[:, i("rho_meas")], cfg["geometry"]["z_s"],
                                weighted=cfg["estimator"]["weighted"])
        print(json.dumps({k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in res.items()},
                         default=lambda v: v.item(), indent=2))


if __name__ == "__main__":
    main()
