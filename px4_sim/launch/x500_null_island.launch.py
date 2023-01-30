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

from ament_index_python.packages import get_package_share_directory
from distutils.dir_util import copy_tree
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import tempfile
import os
from launch.substitutions import LaunchConfiguration


def get_px4_dir():
    return get_package_share_directory('px4_sim')


def seed_rootfs(rootfs):
    px4_dir = get_px4_dir()
    print(f"seeding rootfs at {rootfs} from {px4_dir}")
    copy_tree(px4_dir, rootfs)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(get_px4_dir(), "models")
    rootfs = tempfile.TemporaryDirectory()
    px4_dir = get_px4_dir()

    rc_script = os.path.join(px4_dir, 'etc/init.d-posix/rcS')
    print("using rootfs ", rootfs.name)
    seed_rootfs(rootfs.name)

    os.environ["PX4_GZ_WORLD"] = "null_island"
    os.environ["PX4_GZ_MODEL"] = "x500"
    os.environ["PX4_GZ_MODEL_POSE"] = "-9.7948,-8.31848,2.0,0,0,0"

    run_px4 = ExecuteProcess(
        cmd=['px4', '%s/ROMFS/px4fmu_common' % rootfs.name,
             '-s', rc_script,
             '-i', "id0",
             '-d'],
        cwd=get_px4_dir(),
        output='screen'
    )

    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock')

    world_sdf = os.path.join(get_px4_dir(), "worlds", "null_island.sdf")

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=["udp4", "-p", "8888"],
        output='screen')

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 ' + world_sdf])]),
        run_px4,
        use_sim_time_arg,
        ExecuteProcess(cmd=['QGroundControl.AppImage']),
        micro_ros_agent
    ])
