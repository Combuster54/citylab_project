from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    service_server = Node(
        package='robot_patrol',
        executable='direction_server',
        output='screen')

    patrol_v2 = Node(
        package='robot_patrol',
        executable='patrol_v2',
        output='screen')
    return LaunchDescription([patrol_v2,service_server ])
