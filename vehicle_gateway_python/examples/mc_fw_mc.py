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

import sys
import time

import vehicle_gateway
from vehicle_gateway import ArmingState, ControllerType, FlightMode

vg = vehicle_gateway.init(args=sys.argv, plugin_type='px4')

ready_to_fly = False
while not ready_to_fly:
    # The controller will not enable offboard flight mode unless there is
    # a reasonable offboard mode selected, like a position control setpoint.
    # So let's set one and wait a tiny bit.
    vg.disarm()
    vg.set_offboard_control_mode(ControllerType.POSITION)
    vg.set_local_position_setpoint(0, 0, -5, 1.57)
    time.sleep(0.2)

    # Now we can arm the controller and enable offboard mode
    for arm_attempt in range(0, 5):
        print(f'arm attempt {arm_attempt}...')
        time.sleep(0.2)
        vg.arm()
        vg.set_offboard_mode()
        time.sleep(0.2)
        if vg.get_arming_state() == ArmingState.ARMED and vg.get_flight_mode() == FlightMode.OFFBOARD:
            ready_to_fly = True
            break

        print('vehicle did not go to armed state')
        vg.disarm()

print('vehicle armed in offboard mode. Taking off...')

while vg.get_altitude() > -9.5:
    print(f'altitude: {vg.get_altitude()}')
    sys.stdout.flush()
    time.sleep(0.25)
    vg.set_offboard_control_mode(ControllerType.POSITION)
    vg.set_local_position_setpoint(0, 0, -11, 1.57)

print('spinning...')
for spin_count in range(0, 10):
    vg.set_offboard_control_mode(ControllerType.POSITION)
    vg.set_local_position_setpoint(0, 0, -11, 0)
    time.sleep(0.25)

# while True:
#     vg.set_offboard_control_mode(ControllerType.POSITION)
#     vg.set_local_position_setpoint(0, 0, -3, 1.57)
#     time.sleep(0.1)

print('transitioning to fixed-wing')
for vel_count in range(0, 10):
    vg.set_offboard_control_mode(ControllerType.POSITION)
    vg.set_local_position_setpoint(0, 0, -11, 0)
    time.sleep(0.1)
# while True:
#     #vg.set_air_speed(1.0)
vg.transition_to_fw()
time.sleep(0.1)

going_east = True
speed_setpoint = 12
target_z = -15
while True:
    # vg.set_offboard_control_mode(ControllerType.POSITION)
    # vg.set_local_position_setpoint(0, 0, -11, 0)
    # vg.set_offboard_control_mode(ControllerType.VELOCITY)
    # vg.set_air_speed(1.0)
    x = vg.get_x()
    print(x)
    z_err = target_z - vg.get_altitude()
    vz = -0.2 * z_err
    if going_east and x > 100:
        going_east = False
    elif not going_east and x < -100:
        going_east = True

    vg.set_offboard_control_mode(ControllerType.VELOCITY)
    if going_east:
        vg.set_local_velocity_setpoint(12, 0, vz, 0)
    else:
        vg.set_local_velocity_setpoint(-12, 0, vz, 0)

    time.sleep(0.1)

# time.sleep(30.0)

# print('transitioning back to multicopter')
# vg.transition_to_mc()
#
# time.sleep(15.0)

print('landing')
vg.land()

while vg.get_altitude() < -1.0:
    print(f'altitude: {vg.get_altitude()}')
    sys.stdout.flush()
    time.sleep(0.25)

print('Landed. Disarming...')
sys.stdout.flush()

while vg.get_arming_state() != ArmingState.STANDBY:
    vg.disarm()
    time.sleep(0.5)
vg.destroy()
