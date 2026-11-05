"""MINS on the Iriz 480p rig (ROS 2 version of subscribe_iriz_480p.launch).

Live (wall time):
    ros2 launch mins subscribe_iriz_480p.launch.py
With RViz2 (display_ros2.rviz):
    ros2 launch mins subscribe_iriz_480p.launch.py rviz:=true
From a ROS 2 bag (played with its clock, sim time switched on automatically):
    ros2 launch mins subscribe_iriz_480p.launch.py rosbag:=~/rosbags/lynx_outdoor bag_start_time:=10
"""
import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# base_link in the MINS imu frame
IMU_TO_BASE = ["--x", "-0.131", "--y", "0.0", "--z", "-0.1275"]


def generate_launch_description():
    config = LaunchConfiguration("config")
    config_path = LaunchConfiguration("config_path")
    rosbag = LaunchConfiguration("rosbag")
    bag_start_time = LaunchConfiguration("bag_start_time")
    use_sim_time = LaunchConfiguration("use_sim_time")
    playing_bag = PythonExpression(["'", rosbag, "' != ''"])
    return launch.LaunchDescription([
        DeclareLaunchArgument("config", default_value="Iriz_480p",
                              description="config folder in mins/config"),
        DeclareLaunchArgument("config_path",
                              default_value=PathJoinSubstitution([FindPackageShare("mins"), "config", config,
                                                                  "config.yaml"]),
                              description="master MINS config yaml (overrides config)"),
        DeclareLaunchArgument("rosbag", default_value="",
                              description="ROS 2 bag to play with --clock, empty for live topics"),
        DeclareLaunchArgument("bag_start_time", default_value="0",
                              description="seconds to skip at the start of the bag"),
        DeclareLaunchArgument("use_sim_time", default_value=playing_bag,
                              description="use /clock (defaults to true when a bag is played)"),
        DeclareLaunchArgument("rviz", default_value="false", description="start RViz2 with display_ros2.rviz"),
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
            arguments=["-d", PathJoinSubstitution([FindPackageShare("mins"), "launch", "display_ros2.rviz"])],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_to_base_link",
            arguments=IMU_TO_BASE + ["--frame-id", "imu", "--child-frame-id", "base_link"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        ExecuteProcess(
            cmd=["ros2", "bag", "play", rosbag, "--clock", "--start-offset", bag_start_time],
            output="screen",
            condition=IfCondition(playing_bag),
        ),
    ])
