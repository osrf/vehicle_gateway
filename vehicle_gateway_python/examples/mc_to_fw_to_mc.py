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

import math
import sys
import time

import vehicle_gateway

EARTH_RADIUS = 6378100  # meters

px4_gateway = vehicle_gateway.init(args=sys.argv, plugin_type='px4')


def print_latlon(_vg):
    lat, lon, alt = _vg.get_latlon()
    print(f'Lat: {lat:.5f}, Lon: {lon:.5f}, Alt: {alt:.5f})')


def move_relative(_lat, _lon, _x, _y):
    new_lat = _lat + (_x / EARTH_RADIUS) * (180 / math.pi)
    new_lon = _lon + (_y / EARTH_RADIUS) * (180 / math.pi) / math.cos(_lat * math.pi/180)
    return new_lat, new_lon


print('Arming...')
px4_gateway.arm_sync()
time.sleep(2)  # not sure why this is needed - perhaps some internal state setting

print_latlon(px4_gateway)
home_lat, home_lon, _ = px4_gateway.get_latlon()

print('Takeoff!')
px4_gateway.takeoff()

# give time for the vehicle to move up
time.sleep(5)
print_latlon(px4_gateway)
_, _, home_alt = px4_gateway.get_latlon()

time.sleep(10)

print('Transitioning to fixed-wing...')
px4_gateway.transition_to_fw_sync()
print(f'VTOL state: {px4_gateway.get_vtol_state().name}')

time.sleep(15)

print('Sending new latlon coordinates...')
new_lat, new_lon = move_relative(home_lat, home_lon, 100, -100)
px4_gateway.go_to_latlon(new_lat, new_lon, 600)

time.sleep(30)

print('Go towards land...')
px4_gateway.go_to_latlon(home_lat, home_lon, home_alt)
time.sleep(7)

print('Transitioning to multicopter...')
px4_gateway.transition_to_mc_sync()
print(f'VTOL state: {px4_gateway.get_vtol_state().name}')

print('Go back to land...')
px4_gateway.go_to_latlon(home_lat, home_lon, 200)
time.sleep(20)

print('Landing...')
px4_gateway.land()

time.sleep(30)

print('Disarming...')
px4_gateway.disarm_sync()

px4_gateway.destroy()
print('Demo complete.')
