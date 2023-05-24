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
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from vehicle_gateway_python_helpers.helpers import get_model_pose, get_px4_dir
from vehicle_gateway_python_helpers.helpers import get_px4_process

import os
from launch.substitutions import LaunchConfiguration, PythonExpression
from subprocess import Popen, PIPE
from shlex import split


def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id', default='0')
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value=drone_id,
        description='Set the vehicle ID, default=0')

    dds_domain_id = LaunchConfiguration('dds_domain_id', default='')
    dds_domain_id_arg = DeclareLaunchArgument(
        'dds_domain_id',
        default_value=dds_domain_id,
        description='Set DDS_DOMAIN_ID')

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock')

    frame_name_args = DeclareLaunchArgument(
        'frame_name',
        default_value='',
        description='Frame name included in the SDF world file')

    use_groundcontrol = DeclareLaunchArgument('groundcontrol', default_value='false',
                                              choices=['true', 'false'],
                                              description='Start ground control station.')

    drone_type = LaunchConfiguration('drone_type', default='x500')
    drone_type_args = DeclareLaunchArgument('drone_type', default_value=drone_type,
                                            description='Sim Models (x500, standard_vtol, rc_cessna)')

    sensor_config = LaunchConfiguration('sensor_config', default='stock')
    sensor_config_args = DeclareLaunchArgument('sensor_config', default_value=sensor_config,
                                               description='Sensor configuration from configs_px4 directory')

    world_name = LaunchConfiguration('world_name', default='empty_px4_world')
    world_name_arg = DeclareLaunchArgument('world_name',
                                           default_value=world_name,
                                           description='World name (without .sdf)')

    model_pose = LaunchConfiguration('model_pose', default='')
    model_pose_arg = DeclareLaunchArgument('model_pose',
                                           default_value=model_pose,
                                           description='Model pose (x, y, z, roll, pitch, yaw)')

    [model_pose_x, model_pose_y, model_pose_z,
     model_pose_roll, model_pose_pitch, model_pose_yaw] = get_model_pose(
        LaunchConfiguration('frame_name'),
        LaunchConfiguration('world_name'),
        LaunchConfiguration('model_pose'))

    world_pkgs = get_package_share_directory('vehicle_gateway_worlds')
    px4_dir = get_px4_dir()
    gateway_models_dir = get_package_share_directory('vehicle_gateway_models')

    os.environ['GZ_SIM_RESOURCE_PATH'] = ':' + os.path.join(px4_dir, 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(px4_dir, 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(world_pkgs, 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(gateway_models_dir, 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(gateway_models_dir, 'configs_px4')

    run_px4 = get_px4_process(
        drone_id,
        {'ROS_DOMAIN_ID': LaunchConfiguration('dds_domain_id')})

    wait_spawn = ExecuteProcess(cmd=["sleep", "5"])

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '-p', '8888'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    model_sdf_filename = [
        gateway_models_dir,
        '/configs_px4/',
        LaunchConfiguration('drone_type'),
        '_',
        LaunchConfiguration('sensor_config'),
        '/model.sdf']

    model_name = [LaunchConfiguration('drone_type'), "_", LaunchConfiguration('sensor_config'), "_", LaunchConfiguration('drone_id')]

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

    model_name_env_var = SetEnvironmentVariable('PX4_GZ_MODEL_NAME', model_name)

    autostart_magic_number = PythonExpression([
        '{"x500": 4001, "rc_cessna": 4003, "standard_vtol": 4004}["',
        LaunchConfiguration('drone_type'),
        '"]'])

    autostart_env_var = SetEnvironmentVariable(
        'PX4_SYS_AUTOSTART',
        autostart_magic_number)

    os.environ['PX4_GZ_WORLD'] = ""

    p1 = Popen(split("gz topic -l"), stdout=PIPE)
    p2 = Popen(split("grep -m 1 -e '/world/.*/clock'"), stdin=p1.stdout, stdout=PIPE)
    p3 = Popen(split(r"sed 's/\/world\///g; s/\/clock//g'"), stdin=p2.stdout, stdout=PIPE)
    command_output = p3.stdout.read().decode('utf-8')

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
        dds_domain_id_arg,
        drone_id_arg,
        world_name_arg,
        drone_type_args,
        model_pose_arg,
        frame_name_args,
        sensor_config_args,
        spawn_entity,
        model_name_env_var,
        autostart_env_var,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[wait_spawn],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_spawn,
                on_exit=[run_px4],
            )
        ),
        use_groundcontrol,
        ExecuteProcess(cmd=['QGroundControl.AppImage'],
                       condition=IfCondition(LaunchConfiguration('groundcontrol')))
    ])

    if len(command_output) == 0:
        ld.add_action(micro_ros_agent)
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 ', LaunchConfiguration('world_name'), '.sdf'])]
        ))
    else:
        print('Another gz instance is running, it will only try to spawn the model in gz.')

    return ld
