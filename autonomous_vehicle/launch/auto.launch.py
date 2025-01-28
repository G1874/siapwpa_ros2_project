from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Define the path to the custom world file for the Gazebo simulation.
    world_path = "/home/developer/ros2_ws/src/worlds/custom_city/custom_city.sdf"
    
    # Define the path to the YAML configuration file for the ros_gz_bridge.
    gz_bridge_path = "/home/developer/ros2_ws/src/ros_gz_bridge.yaml"

    # Launch the Gazebo simulation with the custom world.
    gz_sim_world = ExecuteProcess(
        cmd=["gz", "sim", world_path, "-r"]
    )

    # Launch the ROS-Gazebo bridge with the provided configuration file.
    gz_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "--ros-args", "-p", f"config_file:={gz_bridge_path}"
        ]
    )

    # Launch the warp perspective node for transforming coordinates.
    warp_perspective_node = Node(
        package='autonomous_vehicle',
        executable='warp_perspective',
        name='warp_perspective',
        output="screen"
    )

    # Launch the image binarizer node for preprocessing images.
    image_binarizer = Node(
        package='autonomous_vehicle',
        executable="image_binarizer",
        name="image_binarizer",
        output="screen",
    )
    
    # Launch the motion control node for controlling the vehicle's motion.
    motion_control_node = Node(
        package='autonomous_vehicle',
        executable='motion_control',
        name='motion_control',
        output="screen"
    )

    # Launch the image skeletonizer node for extracting line skeletons from images.
    image_skeletonizer = Node(
        package='autonomous_vehicle',
        executable="image_skeletonizer",
        name="image_skeletonizer",
        output="screen",
    )

    # Launch the recognition node for detecting and recognizing objects or signs.
    recognition = Node( 
        package='autonomous_vehicle',
        executable="Recognizing",
        name="Recognizing",
        output="screen",
    )

    # Return the launch description, which includes all the processes to be executed.
    return LaunchDescription([
        gz_sim_world,              # Start Gazebo simulation.
        gz_bridge,                 # Start ROS-Gazebo bridge for communication.
        warp_perspective_node,     # Start warp perspective node for coordinate transformation.
        image_binarizer,           # Start image binarizer for preprocessing.
        motion_control_node,       # Start motion control node to control vehicle.
        image_skeletonizer,        # Start image skeletonizer to process road points.
        recognition                # Start recognition node to detect signs or objects.
    ])
