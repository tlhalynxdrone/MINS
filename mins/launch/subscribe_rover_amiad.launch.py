"""MINS on the AMIAD rover (ROS 2 version of subscribe_rover_amid.launch).

Live (Isaac Sim, sim time), with RViz2 (display_amiad_ros2.rviz) and the Scout + AMIAD rig model
on the MINS estimate:
    ros2 launch mins subscribe_rover_amiad.launch.py
Without RViz2:
    ros2 launch mins subscribe_rover_amiad.launch.py rviz:=false
From a bag (played with its clock):
    ros2 launch mins subscribe_rover_amiad.launch.py rosbag:=~/rosbags/rover_amiad bag_start_time:=10
"""
import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# base_link in the MINS imu frame (pulse80_imu_link of the amiad_description URDF:
# 0.192172 m above base_link, mounted upside down)
IMU_TO_BASE = ["--z", "0.192172", "--roll", "3.141592653589793"]


def generate_launch_description():
    config = LaunchConfiguration("config")
    config_path = LaunchConfiguration("config_path")
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
        DeclareLaunchArgument("config", default_value="rover_amiad",
                              description="config folder in mins/config"),
        DeclareLaunchArgument("config_path",
                              default_value=PathJoinSubstitution([FindPackageShare("mins"), "config", config,
                                                                  "config.yaml"]),
                              description="master MINS config yaml (overrides config)"),
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
        Node(
            package="mins",
            executable="subscribe",
            name="mins_subscribe",
            output="screen",
            parameters=[{"config_path": config_path, "use_sim_time": use_sim_time}],
            on_exit=Shutdown(),
        ),
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
