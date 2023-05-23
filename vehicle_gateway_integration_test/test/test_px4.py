#!/usr/bin/env python3
# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import subprocess
import tempfile

import unittest

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from vehicle_gateway_python_helpers.helpers import get_px4_dir, seed_rootfs

import psutil
import pytest


# This function specifies the processes to be run for our test
@pytest.mark.launch_test
def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    world_pkgs = get_package_share_directory('vehicle_gateway_worlds')

    os.environ['GZ_SIM_RESOURCE_PATH'] = ':' + os.path.join(get_px4_dir(), 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(get_px4_dir(), 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(world_pkgs, 'worlds')

    rootfs = tempfile.TemporaryDirectory()
    px4_dir = get_px4_dir()
    rc_script = os.path.join(px4_dir, 'etc/init.d-posix/rcS')
    print('using rootfs ', rootfs.name)
    seed_rootfs(rootfs.name)

    run_px4 = ExecuteProcess(
        cmd=['px4', '%s/ROMFS/px4fmu_common' % rootfs.name,
             '-s', rc_script,
             '-i', '0',
             '-d'],
        cwd=get_px4_dir(),
        output='screen',
        shell=True)

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '-p', '8888'],
        output='screen')

    if 'SHOW_GZ_GUI' in os.environ and os.environ['SHOW_GZ_GUI']:
        gz_gui_args = ''
    else:
        gz_gui_args = '--headless-rendering -s'
    gz_args = f'{gz_gui_args} -r -v 4 empty_px4_world.sdf'

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
        launch_arguments=[('gz_args', [gz_args])]
    )

    context = {
        'run_px4': run_px4,
        'micro_ros_agent': micro_ros_agent,
        'included_launch': included_launch}

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_type',
            default_value=['x500'],
            description='Vehicle type',
        ),
        DeclareLaunchArgument(
            'script_test',
            default_value=[''],
            description='Script to test',
        ),
        SetEnvironmentVariable('PX4_GZ_MODEL', LaunchConfiguration('vehicle_type')),
        SetEnvironmentVariable('PX4_GZ_WORLD', 'empty_px4_world'),
        SetEnvironmentVariable('PX4_SIM_MODEL', ['gz_', LaunchConfiguration('vehicle_type')]),
        SetEnvironmentVariable('PX4_GZ_MODEL_POSE', '0, 0, 0.3, 0, 0, 0'),
        included_launch,
        run_px4,
        micro_ros_agent,
        KeepAliveProc(),
        # Tell launch to start the test
        ReadyToTest()
    ]), context


class TestFixture(unittest.TestCase):

    def test_arm(self, launch_service, proc_info, proc_output, run_px4):
        try:
            proc_output.assertWaitFor('Ready for takeoff!', process=run_px4,
                                      timeout=100, stream='stdout')

            proc_action = Node(
                package='vehicle_gateway_integration_test',
                executable=LaunchConfiguration('script_test'),
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, proc_action, proc_info, proc_output
            ):
                proc_info.assertWaitForShutdown(process=proc_action, timeout=300)
                launch_testing.asserts.assertExitCodes(proc_info, process=proc_action,
                                                       allowable_exit_codes=[0])
        except AssertionError as e:
            print(e.what())
        finally:
            # shutdown px4
            p = subprocess.Popen('px4-shutdown',
                                 shell=True,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
            p.wait()
            for proc in psutil.process_iter():
                # check whether the process name matches
                if proc.name() == 'ruby' or proc.name() == 'micro_ros_agent':
                    proc.kill()


# These tests are run after the processes in generate_test_description() have shutdown.
@launch_testing.post_shutdown_test()
class TestHelloWorldShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info, run_px4):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info, process=run_px4,
                                               allowable_exit_codes=[0])
