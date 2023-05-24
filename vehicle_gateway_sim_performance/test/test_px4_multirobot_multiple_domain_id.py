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
import pathlib
import sys
import threading

import unittest

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc

import pytest

from vehicle_gateway_python_helpers.helpers import get_px4_dir

sys.path.append(os.path.join(pathlib.Path(__file__).parent.resolve()))  # noqa
from common import create_px4_instance, kill_all_process  # noqa


# This function specifies the processes to be run for our test
@pytest.mark.launch_test
def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    NUMBER_OF_VEHICLES = int(os.environ['NUMBER_OF_VEHICLES'])

    world_pkgs = get_package_share_directory('vehicle_gateway_worlds')
    gtw_models = get_package_share_directory('vehicle_gateway_models')

    os.environ['GZ_SIM_RESOURCE_PATH'] = ':' + os.path.join(get_px4_dir(), 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(get_px4_dir(), 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(world_pkgs, 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(gtw_models, 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(gtw_models, 'configs_px4')

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

    autostart_magic_number = PythonExpression([
        '{"x500": 4001, "rc_cessna": 4003, "standard_vtol": 4004}["',
        LaunchConfiguration('vehicle_type'),
        '"]'])

    autostart_env_var = SetEnvironmentVariable(
        'PX4_SYS_AUTOSTART',
        autostart_magic_number)

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '-p', '8888'],
        output='screen')

    system_metric_collector = Node(
        package='vehicle_gateway_sim_performance',
        executable='system_metric_collector',
        arguments=['system_collector_' + str(NUMBER_OF_VEHICLES) + '_multi_dds_domain.csv'],
        output='screen')

    ld = launch.LaunchDescription([
        system_metric_collector,
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
        SetEnvironmentVariable('PX4_GZ_WORLD', ''),
        SetEnvironmentVariable('PX4_GZ_MODEL', ''),
        included_launch,
        micro_ros_agent,
        KeepAliveProc(),
        # Tell launch to start the test
        ReadyToTest(),
        autostart_env_var
    ])
    context = {'included_launch': included_launch,
               'micro_ros_agent': micro_ros_agent}

    for i in range(1, NUMBER_OF_VEHICLES + 1):
        model_name = [LaunchConfiguration('vehicle_type'), '_stock_', str(i)]
        [run_px4, spawn] = create_px4_instance(
            str(i),
            {'PX4_GZ_MODEL_NAME': model_name,
             'ROS_DOMAIN_ID': str(i)},
            model_name)
        ld.add_action(spawn)
        context['run_px4_' + str(i)] = run_px4

    return ld, context


class TestFixture(unittest.TestCase):

    def test_arm(self, launch_service, proc_info, proc_output):
        NUMBER_OF_VEHICLES = int(os.environ['NUMBER_OF_VEHICLES'])

        try:
            threads = []
            for i in range(1, NUMBER_OF_VEHICLES + 1):
                proc_output.assertWaitFor(f'INFO  [px4] instance: {i}',
                                          timeout=100, stream='stdout')
                proc_output.assertWaitFor('Ready for takeoff!',
                                          timeout=100, stream='stdout')

                def run_test(vehicle_id):
                    proc_action = Node(
                        package='vehicle_gateway_sim_performance',
                        executable=LaunchConfiguration('script_test'),
                        output='screen',
                        additional_env={'ROS_DOMAIN_ID': str(vehicle_id)})
                    with launch_testing.tools.launch_process(
                        launch_service, proc_action, proc_info, proc_output
                    ):
                        proc_info.assertWaitForShutdown(process=proc_action, timeout=300)

                x = threading.Thread(target=run_test, args=(i,))
                x.start()
                threads.append(x)

            for t in threads:
                t.join()
        except AssertionError as e:
            print(e.what())
        finally:
            kill_all_process(NUMBER_OF_VEHICLES)
