"""MINS on the AMIAD rover (ROS 2 version of subscribe_rover_amid.launch), real rig or Isaac Sim.

resolution and num_cameras (ZED X images, 2 per stereo pair) select the config folder in mins/config:
    resolution:=svga | 1080p   real rig (Epson IMU /imu/data, Kalibr calibrations), ZED grab resolution
                               SVGA 960x600 or HD1080 1920x1080: amiad/<resolution>/<folder>
        num_cameras  2: rover_amiad_front                  4: rover_amiad_front_rear
                     6: rover_amiad_front_rear_left        8: rover_amiad_front_rear_left_right
    resolution:=sim            Isaac Sim (SBG IMU, sim cameras): rover_amiad_<num_cameras>_cam
Defaults: resolution 1080p, num_cameras 4; use_sim_time true for sim or a bag, else false.

    ros2 launch amiad_nav onboard.launch.py        # real rig: ZED X, Epson IMU, rover URDF
    ros2 launch mins subscribe_rover_amiad.launch.py resolution:=1080p num_cameras:=8
    ros2 launch mins subscribe_rover_amiad.launch.py resolution:=svga num_cameras:=2
    ros2 launch mins subscribe_rover_amiad.launch.py resolution:=sim num_cameras:=6     # Isaac Sim
Without RViz2 (display_amiad_ros2.rviz, rover B2 + AMIAD short rig model on the MINS estimate):
    ros2 launch mins subscribe_rover_amiad.launch.py rviz:=false
config:=<folder relative to mins/config> or config_path:=<config.yaml> override resolution/num_cameras.
camera_mode:=zed (default, launch_sim_teleop.sh default CAMERA_MODE=zed): the config topics are the ZED
  wrapper gray images /zed_<arm>/zed_node/<eye>/gray/rect/image. With the sim pinhole cameras
  (CAMERA_MODE=ros), remapped to /zed_<arm>/zed_node/<eye>/image_rect_color:
    ros2 launch mins subscribe_rover_amiad.launch.py resolution:=sim camera_mode:=ros
From a bag (played with its clock; use_sim_time then defaults to true):
    ros2 launch mins subscribe_rover_amiad.launch.py rosbag:=~/rosbags/rover_amiad bag_start_time:=10
"""
import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetLaunchConfiguration, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Robot model: rover B2 + AMIAD short rig (rover_description/urdf/rover_amiad.urdf.xacro,
# amiad_short_rig.xacro), same model as amiad_nav onboard.launch.py.
# base_link in the MINS imu frame = Epson /imu/data frame = imu_link of that URDF
# (base_link -> imu_link: xyz 0.03 -0.01 0.139772, no rotation; x forward, z up as found by Kalibr)
IMU_TO_BASE = ["--x", "-0.03", "--y", "0.01", "--z", "-0.139772"]

# config folder in mins/config per resolution and number of cameras (ZED X images, 2 per stereo pair)
REAL_CONFIG_BY_NUM_CAMERAS = {
    "2": "rover_amiad_front",                   # front
    "4": "rover_amiad_front_rear",              # front, rear
    "6": "rover_amiad_front_rear_left",         # front, rear, left
    "8": "rover_amiad_front_rear_left_right",   # front, rear, left, right
}
CONFIGS = {
    # real rig, Kalibr calibrations per ZED grab resolution
    "svga": {n: os.path.join("amiad", "svga", c) for n, c in REAL_CONFIG_BY_NUM_CAMERAS.items()},
    "1080p": {n: os.path.join("amiad", "1080p", c) for n, c in REAL_CONFIG_BY_NUM_CAMERAS.items()},
    # Isaac Sim
    "sim": {n: f"rover_amiad_{n}_cam" for n in REAL_CONFIG_BY_NUM_CAMERAS},
}

ARMS = ["front", "left", "rear", "right"]


def camera_remappings(camera_mode):
    """Config topics (ZED wrapper gray images, camera_mode zed) -> sim pinhole cameras (camera_mode ros)."""
    if camera_mode == "zed":
        return []
    if camera_mode != "ros":
        raise RuntimeError(f"camera_mode must be ros or zed, got '{camera_mode}'")
    return [(f"/zed_{arm}/zed_node/{eye}/gray/rect/image", f"/zed_{arm}/zed_node/{eye}/image_rect_color")
            for arm in ARMS for eye in ("left", "right")]


def mins_node(context):
    """MINS node, its master config from config_path, else config, else resolution + num_cameras."""
    config_path = LaunchConfiguration("config_path").perform(context).strip()
    if not config_path:
        config = LaunchConfiguration("config").perform(context).strip()
        if not config:
            resolution = LaunchConfiguration("resolution").perform(context).strip().lower()
            if resolution not in CONFIGS:
                raise RuntimeError(f"resolution must be one of {list(CONFIGS)}, got '{resolution}'")
            num_cameras = LaunchConfiguration("num_cameras").perform(context).strip()
            if num_cameras not in CONFIGS[resolution]:
                raise RuntimeError(f"num_cameras must be one of {list(CONFIGS[resolution])}, "
                                   f"got '{num_cameras}'")
            config = CONFIGS[resolution][num_cameras]
        config_path = os.path.join(get_package_share_directory("mins"), "config", config, "config.yaml")
    camera_mode = LaunchConfiguration("camera_mode").perform(context).strip().lower()
    return [
        launch.actions.LogInfo(msg=f"[mins] config: {config_path}, camera_mode: {camera_mode}, "
                                   f"use_sim_time: {LaunchConfiguration('use_sim_time').perform(context)}"),
        Node(
            package="mins",
            executable="subscribe",
            name="mins_subscribe",
            output="screen",
            parameters=[{"config_path": config_path,
                         "use_sim_time": LaunchConfiguration("use_sim_time")}],
            remappings=camera_remappings(camera_mode),
            on_exit=Shutdown(),
        ),
    ]


def resolve_use_sim_time(context):
    """use_sim_time empty: true for the Isaac Sim configs or a bag (played with --clock), else false."""
    if LaunchConfiguration("use_sim_time").perform(context).strip():
        return []
    sim = LaunchConfiguration("resolution").perform(context).strip().lower() == "sim"
    bag = bool(LaunchConfiguration("rosbag").perform(context).strip())
    return [SetLaunchConfiguration("use_sim_time", "true" if sim or bag else "false")]


def generate_launch_description():
    rosbag = LaunchConfiguration("rosbag")
    bag_start_time = LaunchConfiguration("bag_start_time")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = IfCondition(LaunchConfiguration("rviz"))
    rviz_config = LaunchConfiguration("rviz_config")
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("urdf_model")]),
        value_type=str)
    return launch.LaunchDescription([
        DeclareLaunchArgument("resolution", default_value="1080p",
                              description="svga | 1080p: real rig configs mins/config/amiad/<resolution> "
                                          "(must match the ZED grab resolution); sim: Isaac Sim configs"),
        DeclareLaunchArgument("num_cameras", default_value="4",
                              description="2 (front), 4 (+ rear), 6 (+ left) or 8 (+ right): selects the "
                                          f"config folder {REAL_CONFIG_BY_NUM_CAMERAS} (overridden by config)"),
        DeclareLaunchArgument("config", default_value="",
                              description="config folder relative to mins/config, e.g. amiad/1080p/rover_amiad_front "
                                          "(overrides resolution and num_cameras)"),
        DeclareLaunchArgument("config_path", default_value="",
                              description="master MINS config yaml (overrides config, resolution and num_cameras)"),
        DeclareLaunchArgument("camera_mode", default_value="zed",
                              description="zed: ZED wrapper /zed_<arm>/zed_node/<eye>/gray/rect/image (config topics), "
                                          "ros: sim images /zed_<arm>/zed_node/<eye>/image_rect_color"),
        DeclareLaunchArgument("rosbag", default_value="",
                              description="bag to play with --clock, empty for live topics"),
        DeclareLaunchArgument("bag_start_time", default_value="0",
                              description="seconds to skip at the start of the bag"),
        DeclareLaunchArgument("use_sim_time", default_value="",
                              description="use /clock; empty: true for resolution:=sim or with rosbag, else false"),
        DeclareLaunchArgument("rviz", default_value="true", description="start RViz2"),
        DeclareLaunchArgument("urdf_model",
                              default_value=PathJoinSubstitution([FindPackageShare("rover_description"), "urdf",
                                                                  "rover_amiad.urdf.xacro"]),
                              description="robot model shown on the MINS estimate (rover B2 + amiad_short_rig); "
                                          "IMU_TO_BASE must match its base_link -> imu_link"),
        DeclareLaunchArgument("rviz_config",
                              default_value=PathJoinSubstitution([FindPackageShare("mins"), "launch",
                                                                  "display_amiad_ros2.rviz"]),
                              description="RViz2 config file"),
        OpaqueFunction(function=resolve_use_sim_time),
        OpaqueFunction(function=mins_node),
        Node(
            package="rviz2",
            executable="rviz2",
            name="mins_rviz",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=rviz,
        ),
        # Robot model on the MINS estimate: the URDF frames prefixed with mins/ (the plain ones
        # belong to the sim's odom -> base_link tree), hung below the MINS imu frame
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="mins",
            parameters=[{"robot_description": robot_description, "frame_prefix": "mins/",
                         "use_sim_time": use_sim_time}],
            remappings=[("joint_states", "/joint_states")],
            condition=rviz,
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="mins_imu_to_base",
            arguments=IMU_TO_BASE + ["--frame-id", "imu", "--child-frame-id", "mins/base_link"],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=rviz,
        ),
        ExecuteProcess(
            cmd=["ros2", "bag", "play", rosbag, "--clock", "--start-offset", bag_start_time],
            output="screen",
            condition=IfCondition(PythonExpression(["'", rosbag, "' != ''"])),
        ),
    ])
