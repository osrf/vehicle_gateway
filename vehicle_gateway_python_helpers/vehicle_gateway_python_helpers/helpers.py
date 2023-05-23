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

from distutils.dir_util import copy_tree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, Substitution, SomeSubstitutionsType
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions

import tempfile
import os
import xml.etree.ElementTree as ET
from typing import List, Literal


def get_px4_dir():
    return get_package_share_directory('px4_sim')


def seed_rootfs(rootfs):
    px4_dir = get_px4_dir()
    print(f'seeding rootfs at {rootfs} from {px4_dir}')
    copy_tree(px4_dir, rootfs)


def get_px4_process(drone_id, additional_env):
    rootfs = tempfile.TemporaryDirectory()
    rc_script = os.path.join(get_px4_dir(), 'etc/init.d-posix/rcS')
    print('using rootfs ', rootfs.name)
    seed_rootfs(rootfs.name)

    run_px4 = ExecuteProcess(
        cmd=['px4', '%s/ROMFS/px4fmu_common' % rootfs.name,
             '-s', rc_script,
             '-i', drone_id,
             '-d'],
        cwd=get_px4_dir(),
        additional_env=additional_env,
        output='screen')

    return run_px4


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
        self.__x = '0.0'
        self.__y = '0.0'
        self.__z = '0.3'
        self.__roll = '0.0'
        self.__pitch = '0.0'
        self.__yaw = '0.0'
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

    def parseCoords(self, strCoords: str,
                    key: Literal["x", "y", "z", "roll", "pitch", "yaw"],
                    strSplit: str):
        x, y, z, roll, pitch, yaw = strCoords.split(strSplit)
        if (key == 'x'):
            return str(x)
        if (key == 'y'):
            return str(y)
        if (key == 'z'):
            return str(z)
        if (key == 'roll'):
            return str(roll)
        if (key == 'pitch'):
            return str(pitch)
        if (key == 'yaw'):
            return str(yaw)
        raise Exception("Not able to parse model pose coordinates")

    def perform(self, context: LaunchContext) -> str:
        frame_name_str = perform_substitutions(context, self.frame_name)
        world_name_str = perform_substitutions(context, self.world_name)
        model_pose_str = perform_substitutions(context, self.model_pose)

        # allow manually specified model_pose param to override lookup
        if model_pose_str != '':
            return self.parseCoords(model_pose_str, self.__coord_name, ', ')

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
        return self.parseCoords('0, 0, 0.3, 0, 0, 0', self.__coord_name, ', ')


def get_model_pose(frame_name, world_name, model_pose):
    model_pose_x = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='x')

    model_pose_y = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='y')

    model_pose_z = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='z')

    model_pose_roll = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='roll')

    model_pose_pitch = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='pitch')

    model_pose_yaw = WorldPoseFromSdfFrame(
        frame_name=frame_name,
        world_name=world_name,
        model_pose=model_pose,
        coor_name='yaw')
    return [model_pose_x, model_pose_y, model_pose_z,
            model_pose_roll, model_pose_pitch, model_pose_yaw]
