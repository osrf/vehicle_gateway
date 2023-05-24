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
from shlex import split
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import psutil

from vehicle_gateway_python_helpers.helpers import get_px4_process


def create_px4_instance(vehicle_id, additional_env, model_name):
    gateway_models_dir = get_package_share_directory('vehicle_gateway_models')

    run_px4 = get_px4_process(
        vehicle_id,
        additional_env,
        ['sleep', str(5 + int(vehicle_id)), '&&'])

    model_sdf_filename = [
        gateway_models_dir,
        '/configs_px4/',
        LaunchConfiguration('vehicle_type'),
        '_',
        'stock',
        '/model.sdf']

    world_sdf_path = os.path.join(
        get_package_share_directory('vehicle_gateway_worlds'),
        'worlds',
        'empty_px4_world.sdf')
    # I couldn't get the libsdformat binding to work as expected due
    # to various troubles. Let's simplify and just treat SDF as
    # regular XML and do an XPath query
    sdf_root = ET.parse(world_sdf_path).getroot()
    frame_node = sdf_root.find(f'.//frame[@name=\"pad_{vehicle_id}\"]')
    if not frame_node:
        raise ValueError(f'Could not find a frame named pad_{vehicle_id}')
    pose_node = frame_node.find('pose')
    pose_str = pose_node.text
    x, y, z, roll, pitch, yaw = pose_str.split(' ')

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', model_sdf_filename,
                   '-name', model_name,
                   '-allow_renaming', 'true',
                   '-x', x,
                   '-y', y,
                   '-z', z,
                   '-R', roll,
                   '-P', pitch,
                   '-Y', yaw],
        on_exit=[run_px4])

    return [run_px4, spawn_entity]


def kill_all_process(NUMBER_OF_VEHICLES):
    for i in range(1, NUMBER_OF_VEHICLES + 1):
        # shutdown px4
        p = subprocess.Popen(split(f'px4-shutdown --instance {i}'),
                             stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE)
        p.wait()
    for proc in psutil.process_iter():
        # check whether the process name matches
        if proc.name() == 'ruby' or \
           proc.name() == 'micro_ros_agent' or \
           proc.name() == 'system_metric_collector':
            proc.kill()
