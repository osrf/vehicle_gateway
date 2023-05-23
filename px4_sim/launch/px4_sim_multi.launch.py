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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from vehicle_gateway_python_helpers.helpers import get_model_pose, get_px4_dir, seed_rootfs

import tempfile
import os
from launch.substitutions import LaunchConfiguration
import yaml


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

                [model_pose_x, model_pose_y, model_pose_z,
                 model_pose_roll, model_pose_pitch, model_pose_yaw] = get_model_pose(
                    vehicle['frame_name'],
                    LaunchConfiguration('world_name'),
                    vehicle['model_pose'])

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
