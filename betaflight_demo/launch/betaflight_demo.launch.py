# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers.on_process_io import OnProcessIO
import os
import time

def get_betaflight_dir():
    return get_package_share_directory('betaflight_sim')


run_virtual_tty = ExecuteProcess(cmd=["socat", "-dd", "pty,link=/tmp/ttyS0,raw,echo=0", "tcp:127.0.0.1:5761"]),


def _run_virtual_tty_check(event):
    """
    Consider betaflight_controller ready when 'bind port 5761 for UART1...' string is printed.

    Launches betaflight_controller node if ready.
    """
    target_str = 'bind port 5761 for UART1'
    if target_str in event.text.decode():
        time.sleep(2)
        return run_virtual_tty


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock')

    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(get_betaflight_dir(), "models")
    os.environ["GZ_SIM_RESOURCE_PATH"] += ":" + os.path.join(get_betaflight_dir(), "worlds")
    os.environ["GZ_SIM_RESOURCE_PATH"] += ":" + os.path.join(get_package_share_directory('vehicle_gateway_models'), "models")
    os.environ["GZ_SIM_RESOURCE_PATH"] += ":" + os.path.join(get_package_share_directory('vehicle_gateway_worlds'), "worlds")
    print("os.environ[GZ_SIM_RESOURCE_PATH]", os.environ["GZ_SIM_RESOURCE_PATH"])
    world_name = LaunchConfiguration('world_name', default='empty_px4_world')
    world_name_arg = DeclareLaunchArgument('world_name',
                                           default_value=world_name,
                                           description='World name')

    run_betaflight_sitl = ExecuteProcess(cmd=['betaflight_SITL.elf', "127.0.0.1"],
                                         cwd=os.path.join(get_betaflight_dir(), "config"),
                                         output='screen')
    return LaunchDescription([
        # Launch gazebo environment
        world_name_arg,
        use_sim_time_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 ', LaunchConfiguration('world_name'), ".sdf"])]
        ),
        run_betaflight_sitl,
        RegisterEventHandler(
            OnProcessIO(
                target_action=run_betaflight_sitl,
                on_stdout=_run_virtual_tty_check,
                on_stderr=_run_virtual_tty_check
            )
        ),
    ])
