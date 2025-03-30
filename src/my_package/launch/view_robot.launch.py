import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    # Get the URDF file path
    urdf_file_path = os.path.join(os.path.expanduser('~'), 'dev_ws', 'example_robot.urdf.xacro')
    
    # Process the URDF file
    robot_description = xacro.process_file(urdf_file_path).toxml()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(os.path.expanduser('~'), 'dev_ws', 'config.rviz')],
            output='screen'
        )
    ])