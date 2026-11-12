"""MINS on the AMIAD rover (ROS 2 version of subscribe_rover_amid.launch).

Live (Isaac Sim, sim time), with RViz2 (display_amiad_ros2.rviz) and the Scout + AMIAD rig model
on the MINS estimate:
    ros2 launch mins subscribe_rover_amiad.launch.py
Without RViz2:
    ros2 launch mins subscribe_rover_amiad.launch.py rviz:=false
num_cameras (ZED X images, 2 per stereo pair) selects the config folder, default 4:
    2: rover_amiad_2_cam (front)          4: rover_amiad_4_cam (front, rear)
    6: rover_amiad_6_cam (+ left)         8: rover_amiad_8_cam (+ left, right)
    ros2 launch mins subscribe_rover_amiad.launch.py num_cameras:=6
camera_mode:=zed (default, launch_sim_teleop.sh default CAMERA_MODE=zed): the config topics are the ZED
  wrapper gray images /zed_<arm>/zed_node/<eye>/gray/rect/image (config intrinsics fx 370.8: ZED SDK
  rectified). With the sim pinhole cameras (CAMERA_MODE=ros), remapped to /zed_<arm>/<eye>/image_rect_color:
    ros2 launch mins subscribe_rover_amiad.launch.py camera_mode:=ros
From a bag (played with its clock):
    ros2 launch mins subscribe_rover_amiad.launch.py rosbag:=~/rosbags/rover_amiad bag_start_time:=10
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

# base_link in the MINS imu frame (pulse80_imu_link of the amiad_description URDF:
# 0.192172 m above base_link, mounted upside down)
IMU_TO_BASE = ["--z", "0.192172", "--roll", "3.141592653589793"]

# config folder in mins/config per number of cameras (ZED X stereo pairs)
CONFIG_BY_NUM_CAMERAS = {
    "2": "rover_amiad_2_cam",  # front
    "4": "rover_amiad_4_cam",  # front, rear
    "6": "rover_amiad_6_cam",  # front, rear, left
    "8": "rover_amiad_8_cam",  # front, rear, left, right
}

ARMS = ["front", "rear", "left", "right"]


def camera_remappings(camera_mode):
    """Config topics (ZED wrapper gray images, camera_mode zed) -> sim pinhole cameras (camera_mode ros)."""
    if camera_mode == "zed":
        return []
    if camera_mode != "ros":
        raise RuntimeError(f"camera_mode must be ros or zed, got '{camera_mode}'")
    return [(f"/zed_{arm}/zed_node/{eye}/gray/rect/image", f"/zed_{arm}/{eye}/image_rect_color")
            for arm in ARMS for eye in ("left", "right")]


def mins_node(context):
    """MINS node, its master config from config_path, else config, else num_cameras."""
    config_path = LaunchConfiguration("config_path").perform(context).strip()
    if not config_path:
        config = LaunchConfiguration("config").perform(context).strip()
        if not config:
            num_cameras = LaunchConfiguration("num_cameras").perform(context).strip()
            if num_cameras not in CONFIG_BY_NUM_CAMERAS:
                raise RuntimeError(f"num_cameras must be one of {list(CONFIG_BY_NUM_CAMERAS)}, "
                                   f"got '{num_cameras}'")
            config = CONFIG_BY_NUM_CAMERAS[num_cameras]
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
        Command(["xacro ", PathJoinSubstitution([FindPackageShare("amiad_description"), "urdf",
                                                 "scout_v2_amiad.urdf.xacro"])]),
        value_type=str)
    return launch.LaunchDescription([
        DeclareLaunchArgument("num_cameras", default_value="4",
                              description="2, 4, 6 or 8: selects the config folder "
                                          f"{CONFIG_BY_NUM_CAMERAS} (overridden by config)"),
        DeclareLaunchArgument("config", default_value="",
                              description="config folder in mins/config (overrides num_cameras)"),
        DeclareLaunchArgument("config_path", default_value="",
                              description="master MINS config yaml (overrides config and num_cameras)"),
        DeclareLaunchArgument("camera_mode", default_value="zed",
                              description="zed: ZED wrapper /zed_<arm>/zed_node/<eye>/gray/rect/image (config topics), "
                                          "ros: sim images /zed_<arm>/<eye>/image_rect_color"),
        DeclareLaunchArgument("rosbag", default_value="",
                              description="bag to play with --clock, empty for live topics"),
        DeclareLaunchArgument("bag_start_time", default_value="0",
                              description="seconds to skip at the start of the bag"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="use /clock (Isaac Sim or bag)"),
        DeclareLaunchArgument("rviz", default_value="true", description="start RViz2"),
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
