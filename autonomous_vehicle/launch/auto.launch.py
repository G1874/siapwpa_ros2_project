from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # world_path = "/home/developer/ros2_ws/src/worlds/custom_city/custom_city.sdf"
    world_path = "/home/developer/ros2_ws/src/worlds/custom_city/custom_city.sdf"
    gz_bridge_path = "/home/developer/ros2_ws/src/ros_gz_bridge.yaml"

    gz_sim_world = ExecuteProcess(
        cmd=["gz", "sim", world_path, "-r"]
    )

    gz_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "--ros-args", "-p", f"config_file:={gz_bridge_path}"
        ]
    )

    warp_perspective_node = Node(
        package='autonomous_vehicle',
        executable='warp_perspective',
        name='warp_perspective',
        output="screen"
    )

    return LaunchDescription([
        gz_sim_world,
        gz_bridge,
        warp_perspective_node
    ])