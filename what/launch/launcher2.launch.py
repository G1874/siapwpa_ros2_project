from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    gz_sim_world = ExecuteProcess(
        cmd=["gz", "sim", "/home/developer/ros2_ws/src/worlds/custom_city/custom_city.sdf", "-r"],
        output="screen",
        shell=False
    )

    gz_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "--ros-args", "-p", "config_file:=ros_gz_bridge.yaml"
        ],
        output="screen"
    )

    image_processing = Node(
        package="what",
        executable="birdseye_transform",
        name="birdseye_transform",
        output="screen",
        parameters=[
            {"parameter_name": "parameter_value"}
        ]
    )

    slider = Node(
        package="what",
        executable="birdseye_slider",
        name="birdseye_slider",
        output="screen",
        parameters=[
            {"parameter_name": "parameter_value"}
        ]
    )

    rqt_image_view_birdseye = ExecuteProcess(
        cmd=["ros2", "run", "rqt_image_view", "rqt_image_view", "/birdseye_view/image"],
        output="screen"
    )

    driver = Node(
        package="what",
        executable="drive_node",
        name="drive_node",
        output="screen",
        parameters=[
            {"parameter_name": "parameter_value"}
        ]
    )


    return LaunchDescription([
        gz_sim_world,
        gz_bridge,
        image_processing,
        slider,
        driver,
        rqt_image_view_birdseye,
    ])
