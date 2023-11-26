import os
import yaml
import unittest
import pytest

import launch_ros
import launch
import launch_testing.actions
import launch_testing.asserts

from launch import LaunchDescription
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# see https://github.com/tylerjw/moveit2/blob/115bfde368e40907f858c43c4e8da4f8c6997d54/moveit_ros/moveit_servo/test/launch/test_servo_pose_tracking.test.py
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_e_stop_test_description(*args,
                                     gtest_name: SomeSubstitutionsType):
    config = os.path.join(
        os.path.dirname(__file__),
        "..",
        'config',
        'asterix_e_stops.yaml'
    )
    e_stop_manager = Node(
        package='e_stop_manager',
        name='e_stop_manager',
        executable='e_stop_manager_node',
        parameters=[config])
    e_stop_manager_gtest = launch_ros.actions.Node(
        executable=PathJoinSubstitution([LaunchConfiguration('test_binary_dir'), gtest_name]),
        output='screen',
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='test_binary_dir',
                                             description='Binary directory of package '
                                                         'containing test executables'),
        e_stop_manager,
        e_stop_manager_gtest,
        launch_testing.actions.ReadyToTest()
    ]), {'e_stop_manager_gtest': e_stop_manager_gtest}


def generate_test_description():
    return generate_e_stop_test_description(gtest_name='e_stop_manager_test')


class TestGTestProcessActive(unittest.TestCase):

    def test_gtest_run_complete(self, proc_info, e_stop_manager_gtest):
        proc_info.assertWaitForShutdown(e_stop_manager_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):

    def test_gtest_pass(self, proc_info, e_stop_manager_gtest):
        launch_testing.asserts.assertExitCodes(proc_info, process=e_stop_manager_gtest)
