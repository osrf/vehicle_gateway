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
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import tempfile
import os
from launch.substitutions import LaunchConfiguration
from typing import List
import yaml
import xml.etree.ElementTree as ET


class WorldPoseFromSdfFrame(Substitution):
    """Substitution that retrieves a frame from SDF."""

    def __init__(self,
                 frame_name: SomeSubstitutionsType,
                 world_name: SomeSubstitutionsType,
                 model_pose: SomeSubstitutionsType,
                 coor_name: str) -> None:
        super().__init__()

        from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
        self.__frame_name = normalize_to_list_of_substitutions(frame_name)
        self.__world_name = normalize_to_list_of_substitutions(world_name)
        self.__model_pose = normalize_to_list_of_substitutions(model_pose)
        self.__x = "0.0"
        self.__y = "0.0"
        self.__z = "0.3"
        self.__roll = "0.0"
        self.__pitch = "0.0"
        self.__yaw = "0.0"
        self.__coord_name = coor_name

    @property
    def model_pose(self) -> List[Substitution]:
        """Getter for model pose."""
        return self.__model_pose

    @property
    def frame_name(self) -> List[Substitution]:
        """Getter for frame name."""
        return self.__frame_name

    @property
    def world_name(self) -> List[Substitution]:
        """Getter for world name."""
        return self.__world_name

    def parseCoords(self, strCoords: str, key: str, strSplit: str):
        x, y, z, roll, pitch, yaw = strCoords.split(strSplit)
        if (self.__coord_name == "x"):
            return str(x)
        if (self.__coord_name == "y"):
            return str(y)
        if (self.__coord_name == "z"):
            return str(z)
        if (self.__coord_name == "roll"):
            return str(roll)
        if (self.__coord_name == "pitch"):
            return str(pitch)
        if (self.__coord_name == "yaw"):
            return str(yaw)
        return "0.0"

    def perform(self, context: LaunchContext) -> str:
        from launch.utilities import perform_substitutions
        frame_name_str = perform_substitutions(context, self.frame_name)
        world_name_str = perform_substitutions(context, self.world_name)
        model_pose_str = perform_substitutions(context, self.model_pose)

        # allow manually specified model_pose param to override lookup
        if model_pose_str != '':
            return self.parseCoords(model_pose_str, self.__coord_name, ", ")

        if frame_name_str != '':
            world_sdf_path = os.path.join(
                get_package_share_directory('vehicle_gateway_worlds'),
                'worlds',
                world_name_str + '.sdf')
            # I couldn't get the libsdformat binding to work as expected due
            # to various troubles. Let's simplify and just treat SDF as
            # regular XML and do an XPath query
            sdf_root = ET.parse(world_sdf_path).getroot()
            frame_node = sdf_root.find(f'.//frame[@name=\'{frame_name_str}\']')
            if not frame_node:
                raise ValueError(f'Could not find a frame named {frame_name_str}')
            pose_node = frame_node.find('pose')
            pose_str = pose_node.text
            # SDFormat stores poses space-separated, but we need them comma-separated
            return self.parseCoords(pose_str, self.__coord_name, " ")

        # default a bit above the origin; vehicle will drop to the ground plane
        return self.parseCoords("0, 0, 0.3, 0, 0, 0", self.__coord_name, ", ")


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

    use_groundcontrol = DeclareLaunchArgument('groundcontrol', default_value='false',
                                              choices=['true', 'false'],
                                              description='Start ground control station.')

    sensor_config = LaunchConfiguration('sensor_config', default='stock')
    sensor_config_args = DeclareLaunchArgument('sensor_config', default_value=sensor_config,
                                               description='Sensor configuration from configs_px4 directory')

    world_name = LaunchConfiguration('world_name', default='empty_px4_world')
    world_name_arg = DeclareLaunchArgument('world_name',
                                           default_value=world_name,
                                           description='World name (without .sdf)')

    world_pkgs = get_package_share_directory('vehicle_gateway_worlds')
    px4_dir = get_px4_dir()
    gateway_models_dir = get_package_share_directory('vehicle_gateway_models')

    os.environ['GZ_SIM_RESOURCE_PATH'] = ':' + os.path.join(px4_dir, 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(px4_dir, 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(world_pkgs, 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(gateway_models_dir, 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(gateway_models_dir, 'configs_px4')

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '-p', '8888'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    os.environ['PX4_GZ_WORLD'] = ""

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    ld = LaunchDescription([
        # Launch gazebo environment
        use_sim_time_arg,
        world_name_arg,
        sensor_config_args,
        bridge,

        micro_ros_agent,
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 ', LaunchConfiguration('world_name'), '.sdf'])]
        ),
        use_groundcontrol,
        ExecuteProcess(cmd=['QGroundControl.AppImage'],
                       condition=IfCondition(LaunchConfiguration('groundcontrol')))
    ])

    magic_number = {"x500": 4001, "rc_cessna": 4003, "standard_vtol": 4004}
    with open(os.environ['MULTIROBOT_CONFIG']) as stream:
        try:
            yaml_data = yaml.safe_load(stream)
            for vehicle in yaml_data:
                (vehicle)
                rootfs = tempfile.TemporaryDirectory()

                rc_script = os.path.join(px4_dir, 'etc/init.d-posix/rcS')
                ('using rootfs ', rootfs.name)
                seed_rootfs(rootfs.name)

                model_name = [vehicle['vehicle_type'], "_", vehicle['sensor_config'], "_", str(vehicle['vehicle_id'])]

                run_px4 = ExecuteProcess(
                    cmd=['px4', '%s/ROMFS/px4fmu_common' % rootfs.name,
                         '-s', rc_script,
                         '-i', str(vehicle['vehicle_id']),
                         '-d'],
                    cwd=px4_dir,
                    additional_env={'PX4_SYS_AUTOSTART': str(magic_number[vehicle['vehicle_type']]),
                                    'ROS_DOMAIN_ID': str(vehicle['dds_domain_id']),
                                    'PX4_GZ_MODEL_NAME': model_name},
                    output='screen')

                model_sdf_filename = [
                    gateway_models_dir,
                    '/configs_px4/',
                    vehicle['vehicle_type'],
                    '_',
                    vehicle['sensor_config'],
                    '/model.sdf']

                model_pose_x = WorldPoseFromSdfFrame(
                    frame_name=vehicle['frame_name'],
                    world_name=LaunchConfiguration('world_name'),
                    model_pose=vehicle['model_pose'],
                    coor_name='x')

                model_pose_y = WorldPoseFromSdfFrame(
                    frame_name=vehicle['frame_name'],
                    world_name=LaunchConfiguration('world_name'),
                    model_pose=vehicle['model_pose'],
                    coor_name='y')

                model_pose_z = WorldPoseFromSdfFrame(
                    frame_name=vehicle['frame_name'],
                    world_name=LaunchConfiguration('world_name'),
                    model_pose=vehicle['model_pose'],
                    coor_name='z')

                model_pose_roll = WorldPoseFromSdfFrame(
                    frame_name=vehicle['frame_name'],
                    world_name=LaunchConfiguration('world_name'),
                    model_pose=vehicle['model_pose'],
                    coor_name='roll')

                model_pose_pitch = WorldPoseFromSdfFrame(
                    frame_name=vehicle['frame_name'],
                    world_name=LaunchConfiguration('world_name'),
                    model_pose=vehicle['model_pose'],
                    coor_name='pitch')

                model_pose_yaw = WorldPoseFromSdfFrame(
                    frame_name=vehicle['frame_name'],
                    world_name=LaunchConfiguration('world_name'),
                    model_pose=vehicle['model_pose'],
                    coor_name='yaw')

                spawn_entity = Node(
                    package='ros_gz_sim',
                    executable='create',
                    output='screen',
                    arguments=['-file', model_sdf_filename,
                               '-name', model_name,
                               '-allow_renaming', 'true',
                               '-x', model_pose_x,
                               '-y', model_pose_y,
                               '-z', model_pose_z,
                               '-R', model_pose_roll,
                               '-P', model_pose_pitch,
                               '-Y', model_pose_yaw])

                ld.add_action(spawn_entity)
                ld.add_action(run_px4)
        except yaml.YAMLError as exc:
            print(exc)

    return ld
