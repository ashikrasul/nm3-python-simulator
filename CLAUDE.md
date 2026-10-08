# CLAUDE.md

Context for working in this repo. Two layers live here:

1. **Upstream NM3 simulator and driver** (Ben Sherlock, MIT): `nm3sim/`, `nm3driver/`, `examples/simple_example.py`, `examples/simple_playback_example.py`. It provides a ZMQ-based real-time acoustic simulator and a serial driver for NM3 acoustic modems. Treat it as a library; avoid editing it.
2. **User research work on beacon localisation**: offline LS trilateration (`examples/lst_square_loc.py`), real-modem ranging (`examples/ranging_example.py`), and a **ROS 2 live localisation package** (`ros2_ws/src/nm3_beacon_sim/`). It works in Gazebo simulation and is designed to run unchanged with real modems.

Untracked scratch scripts not related to localisation: `examples/get_gyro.py` and `examples/live_plot.py` (phone IMU over phyphox).

## Goal of the localisation work
Estimate the position of a static underwater beacon (an NM3 modem) from a surface vessel carrying another NM3 at a fixed transducer depth `z_s`. The vessel pings the beacon every 2 s, and each reply gives a slant range. The position comes from **Method 1, linear least squares with lifting**:

```
rho_k^2 - px_k^2 - py_k^2 = -2 px_k x - 2 py_k y + gamma,   gamma = x^2 + y^2 + (z - z_s)^2
theta = [x, y, gamma] via lstsq;  z = z_s - sqrt(gamma - x^2 - y^2)   (beacon below transducer)
```
This needs at least 3 pings that are **not collinear and are spread in both x and y**.

Modelling assumptions (chosen by the user, same as lst_square):
- fixed `z_s`
- no beacon turnaround delay
- Gaussian range noise σ = 0.5 m
- local ENU frame in metres, z up (beacon z < 0)

## Environment
- Ubuntu 22.04, Python 3.10, **ROS 2 Humble** (`/opt/ros/humble`, `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`).
- Installed: Gazebo Classic 11 (`ros-humble-gazebo-ros-pkgs`), colcon, `teleop_twist_keyboard`, rviz2, rosbag2 (sqlite3).
- Python deps (user site): numpy, matplotlib, pyserial, pyzmq, **tornado**. Tornado was missing; pyzmq's `zmq.eventloop` needs it.
- **Every new shell** must run `source ~/works/nm3-python-simulator/ros2_ws/install/setup.bash`. Otherwise you get `Package 'nm3_beacon_sim' not found`.
- **VS Code is installed as a snap.** Its terminal exports `GTK_PATH`, `LOCPATH`, `GIO_MODULE_DIR`, ... pointing into `/snap/code`. rviz2 then dies with `libpthread.so.0: undefined symbol: __libc_pthread_init`. The originals are kept in `<VAR>_VSCODE_SNAP_ORIG`. `nm3_beacon_sim/launch_utils.py::clean_gui_env()` restores them, and both launch files start rviz2 and gzclient with it. Any new GUI process must do the same, and should use `output="screen"` so crashes are visible.
- Run the upstream examples as modules from the repo root, without `.py`: `python3 -m examples.simple_example`.

## Key upstream APIs reused (do not modify)
- `nm3driver/nm3driver.py::Nm3(input_stream, output_stream)`: works with **any object that has `read()` and `write()`**, such as pyserial.
- `Nm3.send_ping(address, timeout)` (around line 943):
  - It first calls `poll_receiver()`, which drains input until `read()` returns empty.
  - It then writes `$Pnnn`, waits up to `RESPONSE_TIMEOUT = 0.5` s for `$Pnnn\r\n`, then up to `timeout` for `#RnnnTttttt\r\n`.
  - It returns `int(resp[6:11]) * 31.25e-6` seconds, or `-1` on error or timeout (`#TO`).
  - The docstring says "one way time of flight". **This has not been verified on hardware.**
- `examples/lst_square_loc.py::method1_linear_ls(P_xy, rho, z_s, weighted=False)`:
  - Returns `dict(ok, x, gamma, dz2, clamped, rank, cond)`, or `ok=False, msg` when rank < 3.
  - Known failure: if every ping shares one x (or one y), a column of `A` is zero. The column normalisation then produces NaN and **`np.linalg.LinAlgError: SVD did not converge`**. The ROS node guards against this (see `_safe_solve`); the file itself is left untouched.
  - Importing the module sets `matplotlib.use("Agg")`.

## ROS 2 package: `ros2_ws/src/nm3_beacon_sim` (ament_python)

### One pipeline for sim and hardware
`Nm3` only sees a stream. The stream is chosen by `modem.mode` in the JSON config:

| mode | stream given to `Nm3` | pose source |
|---|---|---|
| `sim` | `VirtualNm3Serial`: emulates the vessel modem's serial replies from the true geometry plus noise | Gazebo `/odom` (planar_move) |
| `serial` | `serial.Serial(port, 9600, 8N1, timeout=0.1)`, same as `ranging_example.py` | `/fix` NavSatFix, converted to local ENU (equirectangular around `pose.gps_datum`, or the first fix) |

Everything after the stream is identical in both modes: `send_ping` → `rho = tof * c * tof_scale` → LS → markers and topics.

### Files
| file | role |
|---|---|
| `nm3_beacon_sim/config.py` | `DEFAULTS` + `load_config()` (deep-merge, rejects unknown keys, validates); `add_repo_to_path()` puts the repo root on `sys.path` so `nm3driver` and `examples.lst_square_loc` import; `make_modem_stream()` |
| `nm3_beacon_sim/virtual_modem.py` | `VirtualNm3Serial`: `write(b'$Pnnn')` queues the ack at once and starts an in-flight ping. Each `read()` advances the acoustics (see "Acoustic propagation" below). Reply `#RnnnTttttt`, `ticks = round(rho/(c_modem*tof_scale)/31.25e-6)` clamped to 0..99999. `#TO` if there is no pose or after `NM3_RANGE_TIMEOUT = 4 s`. Replies are queued **only after** a write, because send_ping drains input first. Diagnostics: `last_true_range = (d1+d2)/2`, `last_d1/d2`, `last_pose`, `last_recv_pose`, `last_rtt`. The `clock` argument is injectable (tests use a fake clock). |
| `nm3_beacon_sim/geometry.py` | `horizontal_radius(rho, dz2)`; `circle_points`; `circle_intersections` (returns the closest-approach point when noisy circles miss) |
| `nm3_beacon_sim/pinger_node.py` | the main node (see below) |
| `nm3_beacon_sim/bag_to_csv.py` | `ros2 run nm3_beacon_sim bag_to_csv <run_dir> [--solve]` → `pings.csv`; `--solve` re-runs LS using the run's `config.json` |
| `nm3_beacon_sim/launch_utils.py` | `clean_gui_env()` (snap fix) |
| `launch/sim.launch.py` | reads the JSON. If `sim.use_gazebo`, starts gzserver (via gazebo_ros), gzclient (clean env), and spawns `models/agent.sdf` at the origin and `models/beacon.sdf` at `beacon.true_xyz`. Always starts pinger_node and (optionally) rviz2. If recording is on, it also starts the rosbag recorder. Args: `config_file`, `record` (auto/true/false), `gui`. |
| `launch/playback.launch.py` | rviz2 plus `ros2 bag play` of a recorded run (no Gazebo, no pinger). Args: `run_dir` (default `latest`), `rate`, `loop`. |
| `config/sim.json`, `config/real.json` | all settings (the only config mechanism; no ROS params besides `config_file`) |
| `worlds/beacon.world` | **gravity 0** (the kinematic agent would otherwise fall), translucent water surface at z = 0, axis markers |
| `models/agent.sdf` | box boat with a 2 m transducer pole and `libgazebo_ros_planar_move.so`: `/cmd_vel` in, `/odom` and `odom→base_footprint` TF out |
| `rviz/beacon.rviz` | fixed frame `odom`; MarkerArray `/nm3/markers`, TF, Odometry (off), 10 m grid |
| `test/test_pipeline.py` | ROS-free pytest: VirtualNm3Serial through the real `Nm3.send_ping`, noise statistics, circle intersection, LS convergence |

### Acoustic propagation (virtual modem, `sim.propagation_delay: true`, default)
Real-time two-way travel using the live vessel pose:
```
t_emit = t_send + d1/c_true + beacon_turnaround      d1 = |p_send - b|   (p = (x, y, z_s))
t_recv = t_emit + d2/c_true                           d2 = |p(t_recv) - b|, evaluated when read() sees now >= t_recv
rho_reported = c_modem * (t_recv - t_send)/2 + N(0, sigma) = (c_modem/c_true)(d1+d2)/2 + c_modem*turnaround/2 + noise
```
- `send_ping` really blocks for about `2d/c` (around 50 ms at 37 m).
- `true_speed_of_sound` (default = `modem.speed_of_sound`) models sound-speed error.
- `beacon_turnaround` (default 0, the user's assumption) models an uncompensated beacon delay.
- `propagation_delay: false` restores the old instantaneous reply (static vessel, d1 = d2).

### `pinger_node` behaviour
- **Ping loop:** pinging runs in a **worker thread**, because a real `send_ping` can block for up to `modem.timeout` and must not stall the pose callbacks. Pings are paced start-to-start every `ping.interval`, resyncing after an overrun. The pose is snapshotted at send and again when `send_ping` returns (receive). LS uses `estimator.position` (`midpoint` by default, also `send`/`receive`), because rho is approximately the mean of both legs, which is approximately |p_mid − b|. This works the same with real modems. A failed ping (`-1`) is logged and skipped. Any exception is caught per cycle so the loop never dies.
- **Estimation stages:**
  - **n = 1:** one circle at `depth_prior`.
  - **n ≥ 2 without a fix:** circles plus candidate points from the intersection of the first and latest circles (widest baseline).
  - **n ≥ `min_pings` (3):** `_safe_solve`. It first rejects pings with no spread in x or y (`np.ptp < 1e-3`), then calls `method1_linear_ls` inside try/except. Circles switch from `depth_prior` to the LS `dz2` once a fix exists and is not clamped.
- **Topics published:**
  - `/nm3/markers` (MarkerArray; DELETEALL then redraw, 5 Hz plus on each ping)
  - `/nm3/range` (Float64)
  - `/nm3/estimate` (PointStamped)
  - `/nm3/ping` (Float64MultiArray, one per ping)
- **`/nm3/ping` layout** (`PING_FIELDS`, also in `layout.dim[0].label`): `t_send, x, y, z_s, tof, rho_meas, rho_true(NaN when real or failed), ok, est_x, est_y, est_z, cond, t_recv, x_recv, y_recv, x_used, y_used` (x, y = send pose). New fields are appended so old bags still parse; `bag_to_csv --solve` uses `x_used/y_used` when present.

### Logging (rosbag2)
- Turn it on with `logging.enabled`, or override per run with `record:=true|false`.
- Each run writes `<logging.dir>/nm3_YYYYmmdd_HHMMSS/{config.json, bag/}`. The bag sits in a subfolder because rosbag2 refuses to write into an existing folder.
- The `pose.topic` is always added to the recorded topics.

### Commands
```bash
cd ~/works/nm3-python-simulator/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch nm3_beacon_sim sim.launch.py                       # sim.json: Gazebo + RViz + recording
ros2 run teleop_twist_keyboard teleop_twist_keyboard           # 2nd terminal (needs its own TTY)
ros2 launch nm3_beacon_sim sim.launch.py config_file:=<share>/config/real.json   # real modems + GPS
ros2 launch nm3_beacon_sim playback.launch.py rate:=5.0        # replay newest run
ros2 run nm3_beacon_sim bag_to_csv ~/nm3_logs/<run> --solve
python3 -m pytest -q src/nm3_beacon_sim/test                   # no ROS needed
```
- `--symlink-install` means Python edits take effect without a rebuild. **New files** in `launch/`, `config/` etc. need `colcon build` (setup.py globs them).
- Headless test pattern: a copy of `sim.json` with `viz.rviz=false` and `logging.dir` in a temp folder, plus `gui:=false`. Drive with `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 3.0}, angular: {z: 0.25}}"`. Write output to files, not pipes, because SIGINT through pipes swallows output. The agent must move in **both x and y** before LS can produce a fix.

## Known limitations and open items
- **Propagation delay is modelled** (since 2026-10-08), as described above.
  - The beacon is assumed to reply on a single straight path with constant sound speed: no ray bending, multipath or outliers.
  - The pose is sampled from `/odom` (now 50 Hz in `agent.sdf`), so send/receive poses are quantised to about 20 ms.
- **NM3 ToF convention is unverified** (one-way vs round-trip). Before using real data, range over a known baseline. If the time is round-trip, set `modem.tof_scale: 0.5`. The virtual modem encodes with `c * tof_scale`, so sim and real stay consistent.
- The real-hardware path (`serial` mode, NavSatFix → ENU, real timeouts) has not been run against actual modems yet.
- `ros2 bag play` inside the launch gets no keyboard input. Pause and resume with `ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause` (and `/resume`).

## Reference results
- Sim run `~/nm3_logs/nm3_20261008_111416` (about 10 min, 292 pings, all ok). True beacon (20, −15, −30), `z_s` −2, σ 0.5.
  - Range error: mean −0.07 m, std 0.47 m.
  - First LS fix at ping 49 (the early track lacked x/y spread).
  - Final estimate (19.93, −15.06, −29.96), cond 3.2.
- `lst_square_loc.py` Monte Carlo (circle track, σ 0.3) is the offline baseline for comparison.
