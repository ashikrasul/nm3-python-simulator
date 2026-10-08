# nm3_beacon_sim

Live localisation of a static underwater beacon from a moving vessel, using the NM3 ping
(`Nm3.send_ping`) and the linear least-squares trilateration from
`examples/lst_square_loc.py` (`method1_linear_ls`).

* **Sim:** Gazebo Classic agent driven with `teleop_twist_keyboard`; a virtual NM3 serial port
  answers `$Pnnn` from the true geometry + N(0, σ) range noise.
* **Real:** the same node with the vessel's NM3 on a serial port and GPS for position.
* **RViz:** range circles at transducer depth, two-circle candidates, LS estimate, true beacon.
* **rosbag2:** logging, switchable from the config or the launch command.

## Install and build
```bash
sudo apt install ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions
cd ~/works/nm3-python-simulator/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run (simulation)
```bash
ros2 launch nm3_beacon_sim sim.launch.py                 # uses config/sim.json
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # second terminal; press q to speed up
```

## Run (real modems)
1. Set the vessel modem's port and the beacon modem's address in `config/real.json`.
2. Set the GPS topic in `pose.topic`; it must publish `sensor_msgs/NavSatFix`.
3. Start your GPS driver, then:
```bash
ros2 launch nm3_beacon_sim sim.launch.py config_file:=$(ros2 pkg prefix nm3_beacon_sim)/share/nm3_beacon_sim/config/real.json
```

## Configuration (JSON)
| key | meaning |
|---|---|
| `modem.mode` | `sim` (virtual modem) or `serial` (real NM3 on `serial_port`) |
| `modem.beacon_address` | NM3 address of the beacon modem being pinged |
| `modem.speed_of_sound`, `modem.tof_scale` | `range = tof * c * tof_scale`. Use 0.5 if the modem reports round-trip time |
| `ping.interval` | seconds between ping starts |
| `geometry.z_s` | fixed transducer depth (z up, negative below the surface) |
| `geometry.depth_prior` | beacon depth used to draw circles before LS has a 3D fix |
| `beacon.true_xyz` | true beacon position (required for sim; `null` for real) |
| `sim.sigma_rho`, `sim.seed` | range noise std [m] and RNG seed (sim only) |
| `sim.use_gazebo` | start Gazebo and spawn the agent and beacon |
| `pose.topic`, `pose.type` | `odometry` (`nav_msgs/Odometry`) or `navsatfix` |
| `pose.gps_datum` | `[lat, lon]` origin for local ENU; `null` = first fix |
| `estimator.weighted`, `estimator.min_pings` | options passed to `method1_linear_ls` |
| `logging.enabled` | record a rosbag. Override per run with `record:=true` / `record:=false` |
| `logging.dir`, `logging.storage`, `logging.topics` | output folder, `sqlite3`/`mcap`, topics (the pose topic is always added) |
| `paths.nm3_repo` | repo root containing `nm3driver/`; `null` = auto-detect |

Missing keys fall back to defaults; unknown keys are rejected.

## Logs
Each run creates `<logging.dir>/nm3_YYYYmmdd_HHMMSS/` containing `config.json` and `bag/`.
`/nm3/ping` carries one row per ping:
`t_send, x, y, z_s, tof, rho_meas, rho_true, ok, est_x, est_y, est_z, cond`.
```bash
ros2 run nm3_beacon_sim bag_to_csv ~/nm3_logs/nm3_<stamp> --solve   # pings.csv + LS re-solve
```

## Tests (no ROS needed)
```bash
python3 -m pytest -q src/nm3_beacon_sim/test
```
