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
from vehicle_gateway import ArmingState

px4_gateway = vehicle_gateway.init(args=sys.argv, plugin_type='px4')

print('Arming...')
while px4_gateway.get_arming_state() != ArmingState.ARMED:
    px4_gateway.arm()
    time.sleep(0.1)

print('Takeoff!')
px4_gateway.takeoff()

time.sleep(10)

print('Landing...')
px4_gateway.land()
time.sleep(10)

print('Disarming...')
while px4_gateway.get_arming_state() != ArmingState.STANDBY:
    px4_gateway.disarm()
    time.sleep(0.5)

px4_gateway.destroy()
print('Takeoff and land demo complete.')
