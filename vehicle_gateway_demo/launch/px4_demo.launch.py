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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from vehicle_gateway_python_helpers.helpers import get_model_pose, get_px4_dir
from vehicle_gateway_python_helpers.helpers import get_px4_process

import os


def generate_launch_description():
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
                                            description='Sim Models (x500, rc_cessna, ...)')

    sensor_config = LaunchConfiguration('sensor_config', default='camera')
    sensor_config_args = DeclareLaunchArgument('sensor_config', default_value=sensor_config,
                                               description='Sensor configuration from configs_px4 directory')

    world_name = LaunchConfiguration('world_name', default='aruco_px4_world')
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

    os.environ['GZ_SIM_RESOURCE_PATH'] = ':' + os.path.join(get_px4_dir(), 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(get_px4_dir(), 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(world_pkgs, 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + os.path.join(get_package_share_directory('vehicle_gateway_models'), 'models')

    os.environ['SDF_PATH'] = os.environ['GZ_SIM_RESOURCE_PATH']

    wait_spawn = ExecuteProcess(cmd=["sleep", "5"])
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '-p', '8888'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('vehicle_gateway_demo'), 'config', 'config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    aruco_opencv = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name="aruco_markers",
        parameters=[os.path.join(get_package_share_directory('vehicle_gateway_demo'), 'config', 'single_marker_tracker.yaml')],
        output='screen')

    node_static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=["--x", "0", "--y", "0", "--z", "0.1", "--roll", "0", "--pitch", "1.57", "--yaw", "0.0", "--frame-id", "x500_camera_0/camera_link", "--child-frame-id", "x500_camera_0/camera_link/camera"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    gateway_models_dir = get_package_share_directory('vehicle_gateway_models')

    model_sdf_filename = [
        gateway_models_dir,
        '/configs_px4/',
        LaunchConfiguration('drone_type'),
        '_',
        LaunchConfiguration('sensor_config'),
        '/model.sdf']

    model_name = [LaunchConfiguration('drone_type'), "_", LaunchConfiguration('sensor_config'), "_0"]

    run_px4 = get_px4_process('0', {})

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-file', model_sdf_filename,
                   '-name', model_name,
                   '-allow_renaming', 'true',
                   '-x', model_pose_x,
                   '-y', model_pose_y,
                   '-z', model_pose_z,
                   '-R', model_pose_roll,
                   '-P', model_pose_pitch,
                   '-Y', model_pose_yaw])

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/world/aruco_px4_world/model/x500_camera_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/world/aruco_px4_world/model/x500_camera_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )

    bridge_pose = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/x500_camera_0/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        remappings=[("/model/x500_camera_0/pose", "/tf")],
        output='screen'
    )

    autostart_magic_number = PythonExpression([
        '{"x500": 4001, "rc_cessna": 4003, "standard_vtol": 4004}["',
        LaunchConfiguration('drone_type'),
        '"]'])

    autostart_env_var = SetEnvironmentVariable(
        'PX4_SYS_AUTOSTART',
        autostart_magic_number)

    os.environ['PX4_GZ_WORLD'] = ""
    return LaunchDescription([
        # Launch gazebo environment
        node_static_transform_publisher,
        use_sim_time_arg,
        world_name_arg,
        drone_type_args,
        model_pose_arg,
        frame_name_args,
        sensor_config_args,
        autostart_env_var,
        spawn_entity,
        bridge,
        bridge_pose,
        aruco_opencv,
        SetEnvironmentVariable('PX4_GZ_MODEL_NAME', model_name),
        SetEnvironmentVariable('PX4_SYS_AUTOSTART', '4001'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 ', LaunchConfiguration('world_name'), '.sdf'])]
        ),
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
                       condition=IfCondition(LaunchConfiguration('groundcontrol'))),
        micro_ros_agent,
        rviz2
    ])
