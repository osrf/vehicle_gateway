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

px4_gateway = vehicle_gateway.init(args=sys.argv, plugin_type='px4')

px4_gateway.arm()

time.sleep(1)

px4_gateway.takeoff()

time.sleep(5)

px4_gateway.land()
time.sleep(5)
px4_gateway.destroy()
