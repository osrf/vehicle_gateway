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
from launch import LaunchDescription, Substitution, SomeSubstitutionsType, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import tempfile
import os
from launch.substitutions import LaunchConfiguration
from typing import List
import yaml


class WorldPoseFromYaml(Substitution):
    """Substitution that replaces strings on a given file."""

    def __init__(self,
                 position_name: SomeSubstitutionsType,
                 pos_world_config: SomeSubstitutionsType,
                 model_pose: SomeSubstitutionsType) -> None:
        super().__init__()

        from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self.__position_name = normalize_to_list_of_substitutions(position_name)
        self.__pos_world_config = normalize_to_list_of_substitutions(pos_world_config)
        self.__model_pose = normalize_to_list_of_substitutions(model_pose)

    @property
    def model_pose(self) -> List[Substitution]:
        """Getter for model pose."""
        return self.__model_pose

    @property
    def position_name(self) -> List[Substitution]:
        """Getter for position name."""
        return self.__position_name

    @property
    def pos_world_config(self) -> List[Substitution]:
        """Getter for position world config file."""
        return self.__pos_world_config

    def perform(self, context: LaunchContext) -> str:
        from launch.utilities import perform_substitutions
        position_name_str = perform_substitutions(context, self.position_name)
        pos_world_config_str = perform_substitutions(context, self.pos_world_config)
        model_pose_str = perform_substitutions(context, self.model_pose)

        if pos_world_config_str != '':
            with open(pos_world_config_str) as stream:
                try:
                    yaml_data = yaml.safe_load(stream)
                    if position_name_str in yaml_data:
                        return yaml_data[position_name_str]['position']
                except yaml.YAMLError as exc:
                    print('Not able to read the yaml file: ', position_name_str, exc)
        if model_pose_str != '':
            return model_pose_str
        return '0, 0, 0.3, 0, 0, 0'


def get_px4_dir():
    return get_package_share_directory('px4_sim')


def seed_rootfs(rootfs):
    px4_dir = get_px4_dir()
    print(f'seeding rootfs at {rootfs} from {px4_dir}')
    copy_tree(px4_dir, rootfs)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock')

    config_position_world_args = DeclareLaunchArgument(
        'config_position_world',
        default_value='',
        description='YAML config file with a collection of poses')

    position_name_args = DeclareLaunchArgument(
        'position_name',
        default_value='',
        description='Position name included in the YAML config file')

    use_groundcontrol = DeclareLaunchArgument('groundcontrol', default_value='false',
                                              choices=['true', 'false'],
                                              description='Start ground control station.')

    drone_type = LaunchConfiguration('drone_type', default='x500')
    drone_type_args = DeclareLaunchArgument('drone_type', default_value=drone_type,
                                            description='Sim Models (x500, rc_cessna, ...)')

    world_name = LaunchConfiguration('world_name', default='empty_px4_world')
    world_name_arg = DeclareLaunchArgument('world_name',
                                           default_value=world_name,
                                           description='World name (without .sdf)')

    model_pose = LaunchConfiguration('model_pose', default='0,0,0.3,0,0,0')
    model_pose_arg = DeclareLaunchArgument('model_pose',
                                           default_value=model_pose,
                                           description='Model pose (x, y, z, roll, pitch, yaw)')

    position_name = WorldPoseFromYaml(
        position_name=LaunchConfiguration('position_name'),
        pos_world_config=LaunchConfiguration('config_position_world'),
        model_pose=LaunchConfiguration('model_pose'))

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
             '-i', 'id0',
             '-d'],
        cwd=get_px4_dir(),
        output='screen',
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '-p', '8888'],
        output='screen')

    return LaunchDescription([
        # Launch gazebo environment
        use_sim_time_arg,
        world_name_arg,
        drone_type_args,
        model_pose_arg,
        position_name_args,
        config_position_world_args,
        SetEnvironmentVariable('PX4_GZ_MODEL', LaunchConfiguration('drone_type')),
        SetEnvironmentVariable('PX4_GZ_WORLD', LaunchConfiguration('world_name')),
        SetEnvironmentVariable('PX4_GZ_MODEL_POSE', position_name),
        SetEnvironmentVariable('PX4_SIM_MODEL', ['gz_', LaunchConfiguration('drone_type')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 ', LaunchConfiguration('world_name'), '.sdf'])]
        ),
        run_px4,
        use_groundcontrol,
        ExecuteProcess(cmd=['QGroundControl.AppImage'],
                       condition=IfCondition(LaunchConfiguration('groundcontrol'))),
        micro_ros_agent
    ])
