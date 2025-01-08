from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    world_path = "/home/developer/ros2_ws/src/worlds/custom_city/custom_city.sdf"
    gz_bridge_path = "/home/developer/ros2_ws/src/ros_gz_bridge.yaml"

    gz_sim_world = ExecuteProcess(
        cmd=["gz", "sim", world_path, "-r"],
        output="screen",
        shell=False
    )

    gz_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "--ros-args", "-p", f"config_file:={gz_bridge_path}"
        ],
        output="screen",
        shell=False
    )

    return LaunchDescription([
        gz_sim_world,
        gz_bridge
    ])