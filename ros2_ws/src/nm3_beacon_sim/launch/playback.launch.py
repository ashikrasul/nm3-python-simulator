"""Replay a recorded run in RViz (no Gazebo, no pinger: the recorded markers are shown as-is).

    ros2 launch nm3_beacon_sim playback.launch.py                         # newest run in logging.dir
    ros2 launch nm3_beacon_sim playback.launch.py run_dir:=~/nm3_logs/nm3_20261008_111416
    ros2 launch nm3_beacon_sim playback.launch.py rate:=5.0 loop:=true

Pause / resume from another terminal:
    ros2 service call /rosbag2_player/pause  rosbag2_interfaces/srv/Pause
    ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nm3_beacon_sim.config import load_config
from nm3_beacon_sim.launch_utils import clean_gui_env

PKG = "nm3_beacon_sim"


def _setup(context):
    share = get_package_share_directory(PKG)
    run_dir = os.path.expanduser(LaunchConfiguration("run_dir").perform(context))
    if run_dir in ("", "latest"):
        log_dir = os.path.expanduser(load_config(os.path.join(share, "config", "sim.json"))["logging"]["dir"])
        runs = sorted(d for d in glob.glob(os.path.join(log_dir, "nm3_*")) if os.path.isdir(os.path.join(d, "bag")))
        if not runs:
            raise RuntimeError(f"no recorded runs with a bag/ folder in {log_dir}")
        run_dir = runs[-1]
    bag = os.path.join(run_dir, "bag") if os.path.isdir(os.path.join(run_dir, "bag")) else run_dir
    if not os.path.isfile(os.path.join(bag, "metadata.yaml")):
        raise RuntimeError(f"not a rosbag2 folder: {bag}")

    rate = LaunchConfiguration("rate").perform(context)
    loop = LaunchConfiguration("loop").perform(context).lower() in ("1", "true", "yes", "on")
    cmd = ["ros2", "bag", "play", bag, "--rate", rate]
    if loop:
        cmd.append("--loop")

    return [
        LogInfo(msg=f"[nm3] playing {bag} at x{rate}{' (loop)' if loop else ''}"),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", os.path.join(share, "rviz", "beacon.rviz")], env=clean_gui_env()),
        # small delay so RViz is subscribed before the first markers arrive; end the launch when the bag ends
        ExecuteProcess(cmd=["bash", "-c", "sleep 3 && exec " + " ".join(cmd)], output="screen",
                       emulate_tty=True,
                       on_exit=None if loop else [LogInfo(msg="[nm3] playback finished (close RViz or Ctrl-C)")]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("run_dir", default_value="latest",
                              description="run folder (with bag/) or bag folder; 'latest' = newest run"),
        DeclareLaunchArgument("rate", default_value="1.0", description="playback speed multiplier"),
        DeclareLaunchArgument("loop", default_value="false", description="repeat the bag"),
        OpaqueFunction(function=_setup),
    ])
