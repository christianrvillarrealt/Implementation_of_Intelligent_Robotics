from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(package_dir, 'urdf', 'robot.urdf')
    controller_config = os.path.join(package_dir, 'config', 'controller.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=urdf_file, description='Path to the URDF model'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            output='screen',
            arguments=['-file', urdf_file, '-entity', 'robot']
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # Load controller configurations
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_config],
            output='screen'
        )
    ])
