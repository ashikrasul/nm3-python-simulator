"""Live beacon localisation: ping the beacon via the NM3 driver every `ping.interval` seconds,
solve with lst_square_loc.method1_linear_ls, and visualise range circles / estimate in RViz.

The same node runs against the simulated modem (modem.mode = "sim") or a real NM3 on a serial
port (modem.mode = "serial"); only the stream handed to Nm3 changes.

/nm3/ping (std_msgs/Float64MultiArray), one message per ping, layout PING_FIELDS:
    t_send, x, y, z_s, tof, rho_meas, rho_true (NaN when real), ok,
    est_x, est_y, est_z, cond                    (est_* / cond are NaN until an LS fix exists)
"""
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA, Float64, Float64MultiArray, Header, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray

from .config import add_repo_to_path, load_config, make_modem_stream
from .geometry import circle_intersections, circle_points, horizontal_radius

PING_FIELDS = ["t_send", "x", "y", "z_s", "tof", "rho_meas", "rho_true", "ok",
               "est_x", "est_y", "est_z", "cond"]
EARTH_R = 6378137.0


def _rgba(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def _pt(x, y, z):
    return Point(x=float(x), y=float(y), z=float(z))


class PingerNode(Node):
    def __init__(self):
        super().__init__("nm3_pinger")
        path = self.declare_parameter("config_file", "").value
        if not path:
            raise SystemExit("set the 'config_file' parameter to the JSON config")
        self.cfg = load_config(path)
        add_repo_to_path(self.cfg)
        from nm3driver.nm3driver import Nm3                      # noqa: E402 (repo path added above)
        from examples.lst_square_loc import method1_linear_ls    # noqa: E402
        self._solve = method1_linear_ls

        c = self.cfg
        self.z_s = float(c["geometry"]["z_s"])
        self.dz2_prior = (self.z_s - float(c["geometry"]["depth_prior"])) ** 2
        self.c_eff = c["modem"]["speed_of_sound"] * c["modem"]["tof_scale"]
        self.true_xyz = None if c["beacon"]["true_xyz"] is None else np.asarray(c["beacon"]["true_xyz"], float)
        self.frame = c["viz"]["frame"]

        self._lock = threading.Lock()
        self._pose = None                  # latest (x, y) of the transducer
        self._datum = c["pose"]["gps_datum"]
        self.P, self.rho = [], []           # ping positions (x, y) and measured ranges
        self.res = None                     # latest LS result
        self.trail = []

        self.stream = make_modem_stream(c, self._get_pose)
        self.nm3 = Nm3(input_stream=self.stream, output_stream=self.stream)

        if c["pose"]["type"] == "odometry":
            self.create_subscription(Odometry, c["pose"]["topic"], self._on_odom, 10)
        else:
            self.create_subscription(NavSatFix, c["pose"]["topic"], self._on_fix, 10)

        self.pub_markers = self.create_publisher(MarkerArray, "/nm3/markers", 10)
        self.pub_range = self.create_publisher(Float64, "/nm3/range", 10)
        self.pub_est = self.create_publisher(PointStamped, "/nm3/estimate", 10)
        self.pub_ping = self.create_publisher(Float64MultiArray, "/nm3/ping", 10)
        self.create_timer(0.2, self._publish_markers)   # keeps agent marker live between pings

        self._running = True
        self._worker = threading.Thread(target=self._ping_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"config {c['_path']}: mode={c['modem']['mode']} beacon_addr={c['modem']['beacon_address']} "
            f"interval={c['ping']['interval']}s z_s={self.z_s} true_beacon={c['beacon']['true_xyz']}")

    # ------------------------------------------------------------------ pose input
    def _get_pose(self):
        with self._lock:
            return self._pose

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        with self._lock:
            self._pose = (p.x, p.y)

    def _on_fix(self, msg: NavSatFix):
        if msg.status.status < 0 or not math.isfinite(msg.latitude):
            return
        if self._datum is None:
            self._datum = [msg.latitude, msg.longitude]
            self.get_logger().info(f"GPS datum set to first fix {self._datum}")
        lat0, lon0 = map(math.radians, self._datum)
        x = EARTH_R * (math.radians(msg.longitude) - lon0) * math.cos(lat0)   # local ENU (equirectangular)
        y = EARTH_R * (math.radians(msg.latitude) - lat0)
        with self._lock:
            self._pose = (x, y)

    # ------------------------------------------------------------------ ping loop (worker thread)
    def _ping_loop(self):
        m = self.cfg["modem"]
        interval = float(self.cfg["ping"]["interval"])
        next_t = time.time()
        while self._running and rclpy.ok():
            while self._running and time.time() < next_t:
                time.sleep(0.02)
            if not self._running:
                break
            next_t += interval
            if time.time() > next_t:                      # overran (e.g. real-modem timeout): resync
                next_t = time.time() + interval
            pose = self._get_pose()
            if pose is None:
                self.get_logger().warn("no pose yet, skipping ping", throttle_duration_sec=5.0)
                continue
            t_send = time.time()
            try:
                tof = self.nm3.send_ping(int(m["beacon_address"]), timeout=float(m["timeout"]))
                self._handle_ping(t_send, pose, tof)
            except Exception as e:                         # never let one bad ping stop the loop
                self.get_logger().error(f"ping cycle failed: {e!r}")

    def _handle_ping(self, t_send, pose, tof):
        ok = tof is not None and tof >= 0
        rho = tof * self.c_eff if ok else float("nan")
        rho_true = getattr(self.stream, "last_true_range", float("nan")) if ok else float("nan")
        if ok:
            with self._lock:
                self.P.append(pose)
                self.rho.append(rho)
                self.trail.append(pose)
                n = len(self.P)
                if n >= self.cfg["estimator"]["min_pings"]:
                    self.res = self._safe_solve(np.asarray(self.P), np.asarray(self.rho))
                res = self.res
            self.pub_range.publish(Float64(data=float(rho)))
        else:
            with self._lock:
                n, res = len(self.P), self.res
            self.get_logger().warn(f"ping to {self.cfg['modem']['beacon_address']} failed or timed out")

        est = res["x"] if res is not None and res["ok"] else np.full(3, np.nan)
        cond = res["cond"] if res is not None else float("nan")
        if res is not None and res["ok"]:
            self.pub_est.publish(PointStamped(header=self._header(),
                                              point=_pt(*est)))
        self._publish_ping(t_send, pose, tof if ok else float("nan"), rho, rho_true, ok, est, cond)
        self._log_ping(n, pose, rho, rho_true, ok, res)
        self._publish_markers()

    def _safe_solve(self, P, rho):
        """method1_linear_ls, guarded against degenerate geometry (e.g. all pings at one x or y,
        which makes a column of A zero and the SVD fail)."""
        if np.ptp(P[:, 0]) < 1e-3 or np.ptp(P[:, 1]) < 1e-3:
            return dict(ok=False, rank=1, cond=float("inf"), msg="pings not spread in x and y")
        try:
            return self._solve(P, rho, self.z_s, weighted=self.cfg["estimator"]["weighted"])
        except (np.linalg.LinAlgError, ValueError) as e:
            return dict(ok=False, rank=0, cond=float("inf"), msg=f"LS failed: {e}")

    def _publish_ping(self, t_send, pose, tof, rho, rho_true, ok, est, cond):
        msg = Float64MultiArray()
        msg.layout.dim = [MultiArrayDimension(label=",".join(PING_FIELDS), size=len(PING_FIELDS),
                                              stride=len(PING_FIELDS))]
        msg.data = [float(v) for v in (t_send, pose[0], pose[1], self.z_s, tof, rho, rho_true,
                                       1.0 if ok else 0.0, *est, cond)]
        self.pub_ping.publish(msg)

    def _log_ping(self, n, pose, rho, rho_true, ok, res):
        s = f"#{n} pos=({pose[0]:.2f},{pose[1]:.2f}) "
        s += f"rho={rho:.2f}m" if ok else "rho=FAIL"
        if math.isfinite(rho_true):
            s += f" (true {rho_true:.2f})"
        if res is not None:
            if res["ok"]:
                e = res["x"]
                s += f" | est=({e[0]:.2f},{e[1]:.2f},{e[2]:.2f}) cond={res['cond']:.1f}"
                if self.true_xyz is not None:
                    d = e - self.true_xyz
                    s += f" |e|H={math.hypot(d[0], d[1]):.2f} |e|z={abs(d[2]):.2f}"
                if res["clamped"]:
                    s += " [depth clamped]"
            else:
                s += f" | LS: {res['msg']}"
        self.get_logger().info(s)

    # ------------------------------------------------------------------ markers
    def _header(self):
        return Header(frame_id=self.frame, stamp=self.get_clock().now().to_msg())

    def _marker(self, ns, mid, mtype, color, scale):
        mk = Marker(header=self._header(), ns=ns, id=mid, type=mtype, action=Marker.ADD, color=color)
        mk.pose.orientation.w = 1.0
        mk.scale.x, mk.scale.y, mk.scale.z = (float(v) for v in scale)
        return mk

    def _publish_markers(self):
        with self._lock:
            P, rho, res, pose = list(self.P), list(self.rho), self.res, self._pose
            trail = list(self.trail)
        z = self.z_s
        arr = MarkerArray()
        arr.markers.append(Marker(header=self._header(), action=Marker.DELETEALL))

        have_fix = res is not None and res["ok"]
        dz2 = res["dz2"] if have_fix and not res["clamped"] else self.dz2_prior

        # range circles (horizontal, at transducer depth), newest highlighted
        max_c = int(self.cfg["viz"]["max_circles"])
        first = max(0, len(P) - max_c)
        for i in range(first, len(P)):
            r = horizontal_radius(rho[i], dz2)
            if not math.isfinite(r):
                continue
            latest = i == len(P) - 1
            mk = self._marker("circles", i, Marker.LINE_STRIP,
                              _rgba(1.0, 0.85, 0.1, 1.0) if latest else _rgba(0.2, 0.8, 0.3, 0.35),
                              (0.12 if latest else 0.05, 0, 0))
            mk.points = [_pt(px, py, z) for px, py in circle_points(P[i][0], P[i][1], r)]
            arr.markers.append(mk)

        # ping positions
        if P:
            mk = self._marker("pings", 0, Marker.SPHERE_LIST, _rgba(0.9, 0.2, 0.2), (0.4, 0.4, 0.4))
            mk.points = [_pt(px, py, z) for px, py in P]
            arr.markers.append(mk)
        if len(trail) > 1:
            mk = self._marker("trail", 0, Marker.LINE_STRIP, _rgba(0.9, 0.2, 0.2, 0.6), (0.05, 0, 0))
            mk.points = [_pt(px, py, z) for px, py in trail]
            arr.markers.append(mk)

        # two-circle candidates until LS has a fix (first vs latest ping: widest baseline)
        if len(P) >= 2 and not have_fix:
            cands = circle_intersections(P[0], horizontal_radius(rho[0], dz2),
                                         P[-1], horizontal_radius(rho[-1], dz2))
            z_c = self.z_s - math.sqrt(self.dz2_prior)
            for k, (cx, cy) in enumerate(cands):
                mk = self._marker("candidates", k, Marker.SPHERE, _rgba(1.0, 0.5, 0.0, 0.9), (1.0, 1.0, 1.0))
                mk.pose.position = _pt(cx, cy, z_c)
                arr.markers.append(mk)
                ln = self._marker("candidates_line", k, Marker.LINE_LIST, _rgba(1.0, 0.5, 0.0, 0.5), (0.05, 0, 0))
                ln.points = [_pt(cx, cy, z), _pt(cx, cy, z_c)]
                arr.markers.append(ln)

        # LS estimate
        if have_fix:
            ex, ey, ez = res["x"]
            mk = self._marker("estimate", 0, Marker.SPHERE, _rgba(1.0, 0.45, 0.0), (1.2, 1.2, 1.2))
            mk.pose.position = _pt(ex, ey, ez)
            arr.markers.append(mk)
            ln = self._marker("estimate_line", 0, Marker.LINE_LIST, _rgba(1.0, 0.45, 0.0, 0.7), (0.06, 0, 0))
            ln.points = [_pt(ex, ey, z), _pt(ex, ey, ez)]
            arr.markers.append(ln)
            mk = self._marker("estimate_top", 0, Marker.CYLINDER, _rgba(1.0, 0.45, 0.0, 0.8), (0.8, 0.8, 0.05))
            mk.pose.position = _pt(ex, ey, z)
            arr.markers.append(mk)

        # true beacon
        if self.true_xyz is not None:
            bx, by, bz = self.true_xyz
            mk = self._marker("true_beacon", 0, Marker.SPHERE, _rgba(0.1, 0.4, 1.0, 0.8), (1.0, 1.0, 1.0))
            mk.pose.position = _pt(bx, by, bz)
            arr.markers.append(mk)
            ln = self._marker("true_beacon_line", 0, Marker.LINE_LIST, _rgba(0.1, 0.4, 1.0, 0.4), (0.04, 0, 0))
            ln.points = [_pt(bx, by, 0.0), _pt(bx, by, bz)]
            arr.markers.append(ln)

        # agent + status text
        if pose is not None:
            mk = self._marker("agent", 0, Marker.CUBE, _rgba(0.9, 0.9, 0.9), (1.2, 0.6, 0.3))
            mk.pose.position = _pt(pose[0], pose[1], 0.0)
            arr.markers.append(mk)
            ln = self._marker("agent_transducer", 0, Marker.LINE_LIST, _rgba(0.9, 0.9, 0.9), (0.05, 0, 0))
            ln.points = [_pt(pose[0], pose[1], 0.0), _pt(pose[0], pose[1], z)]
            arr.markers.append(ln)
            tx = self._marker("status", 0, Marker.TEXT_VIEW_FACING, _rgba(1, 1, 1), (0, 0, 1.0))
            tx.pose.position = _pt(pose[0], pose[1], 3.0)
            tx.text = self._status_text(len(P), res)
            arr.markers.append(tx)

        self.pub_markers.publish(arr)

    def _status_text(self, n, res):
        if n == 0:
            return "waiting for first ping"
        if res is None:
            return f"{n} ping(s): circles at prior depth {self.cfg['geometry']['depth_prior']} m"
        if not res["ok"]:
            return f"{n} pings: {res['msg']} - move / turn!"
        e = res["x"]
        s = f"{n} pings  est=({e[0]:.1f}, {e[1]:.1f}, {e[2]:.1f})  cond={res['cond']:.0f}"
        if self.true_xyz is not None:
            d = e - self.true_xyz
            s += f"\n|e|H={math.hypot(d[0], d[1]):.2f} m  |e|z={abs(d[2]):.2f} m"
        return s

    def destroy_node(self):
        self._running = False
        self._worker.join(timeout=float(self.cfg["modem"]["timeout"]) + 1.0)
        close = getattr(self.stream, "close", None)
        if close:
            close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PingerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:                                   # launch may deliver a second SIGINT during teardown
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
