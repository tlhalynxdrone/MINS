"""MINS on the real AMIAD rover, ZED X at HD1080 (1920x1080), Epson IMU /imu/data.

Configs in mins/config/amiad/1080p (Kalibr camera-IMU calibrations at 1080p), selected by cameras:
    front       rover_amiad_front        1 stereo pair (cam0/1 front)
    front_rear  rover_amiad_front_rear   2 stereo pairs (cam0/1 front, cam2/3 rear)   default

    ros2 launch amiad_nav onboard.launch.py        # ZED X (HD1080), Epson IMU, rover URDF
    ros2 launch mins subscribe_rover_amiad_1080p.launch.py                      # front + rear
    ros2 launch mins subscribe_rover_amiad_1080p.launch.py cameras:=front       # front only
Without RViz2:
    ros2 launch mins subscribe_rover_amiad_1080p.launch.py rviz:=false
Topics: ZED wrapper gray images /zed_<arm>/zed_node/<eye>/gray/rect/image (camera_mode:=zed, default);
camera_mode:=ros remaps them to /zed_<arm>/zed_node/<eye>/image_rect_color.
From a bag (played with its clock, so use_sim_time:=true):
    ros2 launch mins subscribe_rover_amiad_1080p.launch.py rosbag:=~/rosbags/rover_amiad use_sim_time:=true bag_start_time:=10
"""
import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
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

# config folder in mins/config per camera combination (ZED X stereo pairs), 1080p calibrations
CONFIG_DIR = os.path.join("amiad", "1080p")
CONFIG_BY_CAMERAS = {
    "front": "rover_amiad_front",            # front pair: cam0/1
    "front_rear": "rover_amiad_front_rear",  # front, rear pairs: cam0/1, cam2/3
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
    """MINS node, its master config from config_path, else config, else cameras."""
    config_path = LaunchConfiguration("config_path").perform(context).strip()
    if not config_path:
        config = LaunchConfiguration("config").perform(context).strip()
        if not config:
            cameras = LaunchConfiguration("cameras").perform(context).strip().lower()
            if cameras not in CONFIG_BY_CAMERAS:
                raise RuntimeError(f"cameras must be one of {list(CONFIG_BY_CAMERAS)}, got '{cameras}'")
            config = os.path.join(CONFIG_DIR, CONFIG_BY_CAMERAS[cameras])
        config_path = os.path.join(get_package_share_directory("mins"), "config", config, "config.yaml")
    camera_mode = LaunchConfiguration("camera_mode").perform(context).strip().lower()
    return [
        launch.actions.LogInfo(msg=f"[mins] config: {config_path}, camera_mode: {camera_mode}"),
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
        DeclareLaunchArgument("cameras", default_value="front_rear",
                              description="camera combination, selects the config folder in "
                                          f"mins/config/{CONFIG_DIR}: {CONFIG_BY_CAMERAS} (overridden by config)"),
        DeclareLaunchArgument("config", default_value="",
                              description="config folder relative to mins/config, e.g. amiad/1080p/rover_amiad_front "
                                          "(overrides cameras)"),
        DeclareLaunchArgument("config_path", default_value="",
                              description="master MINS config yaml (overrides config and cameras)"),
        DeclareLaunchArgument("camera_mode", default_value="zed",
                              description="zed: ZED wrapper /zed_<arm>/zed_node/<eye>/gray/rect/image (config topics), "
                                          "ros: sim images /zed_<arm>/zed_node/<eye>/image_rect_color"),
        DeclareLaunchArgument("rosbag", default_value="",
                              description="bag to play with --clock, empty for live topics"),
        DeclareLaunchArgument("bag_start_time", default_value="0",
                              description="seconds to skip at the start of the bag"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="real rig: false; true with rosbag (bag played with --clock)"),
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
