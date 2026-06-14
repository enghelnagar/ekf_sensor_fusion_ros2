import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    Main Launch file for the EKF Experiment.
    It launches the Gazebo simulation provided by the professor,
    starts the custom trajectory driver, and runs the EKF fusion node.
    """
    
    # 1. Path to the professor's Gazebo simulation launch file
    gazebo_pkg_dir = get_package_share_directory('bme_gazebo_sensors')
    gazebo_launch_path = os.path.join(gazebo_pkg_dir, 'launch', 'spawn_robot_ex.launch.py')
    
    # Include the Gazebo simulation
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
 
    )

    # 2. Custom Gazebo Driver Node (Figure-8 Path)
    driver_node = Node(
        package='ekf_experiment',
        executable='gazebo_driver',
        name='gazebo_driver_node',
        output='screen'
    )

    # 3. Custom EKF Fusion Node
    ekf_node = Node(
        package='ekf_experiment',
        executable='ekf_fusion',
        name='ekf_fusion_node',
        output='screen'
    )

    # 4. Ground Truth Node
    kinematic_node = Node(
        package='ekf_experiment',
        executable='kinematic_sim', 
        name='kinematic_sim_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_simulation,
        #driver_node,
        ekf_node,
        kinematic_node
    ])