"""Launch the live NM3 beacon localisation.

    ros2 launch nm3_beacon_sim sim.launch.py                                   # sim.json (Gazebo + sim modem)
    ros2 launch nm3_beacon_sim sim.launch.py config_file:=/path/to/real.json   # real modems, no Gazebo
    ros2 launch nm3_beacon_sim sim.launch.py record:=false                     # override logging.enabled

Drive the agent from a second terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os
import shutil
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nm3_beacon_sim.config import load_config
from nm3_beacon_sim.launch_utils import clean_gui_env

PKG = "nm3_beacon_sim"


def _setup(context):
    share = get_package_share_directory(PKG)
    cfg_path = os.path.expanduser(LaunchConfiguration("config_file").perform(context))
    cfg = load_config(cfg_path)
    record_arg = LaunchConfiguration("record").perform(context).lower()
    record = cfg["logging"]["enabled"] if record_arg in ("", "auto") else record_arg in ("1", "true", "yes", "on")
    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes", "on")
    actions = [LogInfo(msg=f"[nm3] config: {cfg_path}  mode={cfg['modem']['mode']}  record={record}")]

    if cfg["sim"]["use_gazebo"]:
        gazebo_ros = get_package_share_directory("gazebo_ros")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gazebo.launch.py")),
            launch_arguments={"world": os.path.join(share, "worlds", "beacon.world"),
                              "gui": "false"}.items()))
        if gui:
            actions.append(ExecuteProcess(cmd=["gzclient"], env=clean_gui_env(), output="screen"))
        actions.append(Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
                            arguments=["-entity", "agent", "-file", os.path.join(share, "models", "agent.sdf"),
                                       "-x", "0", "-y", "0", "-z", "0"]))
        if cfg["beacon"]["true_xyz"] is not None:
            bx, by, bz = (str(v) for v in cfg["beacon"]["true_xyz"])
            actions.append(Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
                                arguments=["-entity", "beacon", "-file", os.path.join(share, "models", "beacon.sdf"),
                                           "-x", bx, "-y", by, "-z", bz]))

    actions.append(Node(package=PKG, executable="pinger_node", name="nm3_pinger", output="screen",
                        parameters=[{"config_file": cfg_path}]))

    if cfg["viz"]["rviz"]:
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
                            arguments=["-d", os.path.join(share, "rviz", "beacon.rviz")],
                            env=clean_gui_env()))

    if record:
        log = cfg["logging"]
        run_dir = os.path.join(os.path.expanduser(log["dir"]), time.strftime("nm3_%Y%m%d_%H%M%S"))
        os.makedirs(run_dir, exist_ok=True)
        shutil.copyfile(cfg_path, os.path.join(run_dir, "config.json"))
        topics = list(dict.fromkeys(log["topics"] + [cfg["pose"]["topic"]]))   # always log the pose source
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "-s", log["storage"], "-o", os.path.join(run_dir, "bag"), *topics],
            output="screen"))
        actions.append(LogInfo(msg=f"[nm3] recording to {run_dir}/bag : {' '.join(topics)}"))
    return actions


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory(PKG), "config", "sim.json")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_cfg, description="JSON experiment config"),
        DeclareLaunchArgument("record", default_value="auto",
                              description="true/false to override logging.enabled; auto = use the JSON"),
        DeclareLaunchArgument("gui", default_value="true", description="show the Gazebo client window"),
        OpaqueFunction(function=_setup),
    ])
