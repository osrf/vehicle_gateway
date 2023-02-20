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
from vehicle_gateway import ArmingState, FlightMode, ControllerType

px4_gateway = vehicle_gateway.init(args=sys.argv, plugin_type='px4')

print(px4_gateway.get_arming_state())

px4_gateway.disarm()

_start_time = time.perf_counter()
offboard_setpoint_counter_ = 0
while 1:
    current_time = time.perf_counter()
    elapsed_time = current_time - _start_time
    if elapsed_time >= 1:
        if offboard_setpoint_counter_ < 6:
            offboard_setpoint_counter_ += 1

        if offboard_setpoint_counter_ == 5:
            while px4_gateway.get_flight_mode() != FlightMode.OFFBOARD:
                px4_gateway.set_offboard_mode()
                print('Setting offboard mode')
                time.sleep(0.01)
            while px4_gateway.get_arming_state() != ArmingState.ARMED:
                px4_gateway.arm()
                print('Arming')
                time.sleep(0.01)

        px4_gateway.set_offboard_control_mode(ControllerType.POSITION)
        px4_gateway.set_local_position_setpoint(0, -5, -5, 0)

        _start_time = current_time
    time.sleep(0.1)
