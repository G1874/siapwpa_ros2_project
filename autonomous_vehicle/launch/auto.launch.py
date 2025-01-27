from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
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

    image_binarizer = Node(
        package='autonomous_vehicle',
        executable="image_binarizer",
        name="image_binarizer",
        output="screen",
    )
    
    motion_control_node = Node(
        package='autonomous_vehicle',
        executable='motion_control',
        name='motion_control',
        output="screen"
    )

    image_skeletonizer = Node(
        package='autonomous_vehicle',
        executable="image_skeletonizer",
        name="image_skeletonizer",
        output="screen",
    )

    recognition = Node( 
        package='autonomous_vehicle',
        executable="Recognizing",
        name="Recognizing",
        output="screen",
    )

    return LaunchDescription([
        gz_sim_world,
        gz_bridge,
        warp_perspective_node,
        image_binarizer,
        motion_control_node,
        image_skeletonizer,
        recognition
    ])