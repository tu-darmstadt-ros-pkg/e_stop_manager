import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('e_stop_manager'),
        'config',
        'asterix_e_stops.yaml'
    )

    node = Node(
        package='e_stop_manager',
        name='e_stop_manager',
        executable='e_stop_manager_node',
        parameters=[config]
    )
    ld.add_action(node)
    return ld
