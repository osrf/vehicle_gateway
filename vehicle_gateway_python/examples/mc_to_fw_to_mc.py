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

TARGET_ALTITUDE = 50  # meters above takeoff point

EARTH_RADIUS = 6378100  # meters

px4_gateway = vehicle_gateway.init(args=sys.argv, plugin_type='px4')


def print_latlon(_vg):
    lat, lon, alt_amsl = _vg.get_latlon()
    print(f'Lat: {lat:.5f}, Lon: {lon:.5f}, Alt_AMSL: {alt_amsl:.5f})')


def move_relative_meters(_lat, _lon, _x, _y):
    new_lon = _lon + (_x / EARTH_RADIUS) * (180 / math.pi) \
        / math.cos(_lat * math.pi/180)
    new_lat = _lat + (_y / EARTH_RADIUS) * (180 / math.pi)
    return new_lat, new_lon


def calc_distance_latlon(lat_1, lon_1, lat_2, lon_2):
    # This uses a planar approximation, not the real trigonometry. This
    # approximation is just fine for short distances, which is all we're
    # currently considering.
    dx = (EARTH_RADIUS * math.pi / 180) * (lon_2 - lon_1) * math.cos(lat_1)
    dy = (EARTH_RADIUS * math.pi / 180) * (lat_2 - lat_1)
    return math.sqrt(dx * dx + dy * dy)


print('Arming...')
px4_gateway.arm_sync()
time.sleep(2)  # not sure why... perhaps some internal state setting?

print_latlon(px4_gateway)
home_lat, home_lon, home_alt_amsl = px4_gateway.get_latlon()

print('Takeoff!')
px4_gateway.takeoff()
print_latlon(px4_gateway)

for t in range(0, 10):
    lat, lon, alt_amsl = px4_gateway.get_latlon()
    dalt = alt_amsl - home_alt_amsl
    print(f'takeoff delay: {dalt}')
    time.sleep(1)

px4_gateway.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
while True:
    time.sleep(1)
    lat, lon, alt_amsl = px4_gateway.get_latlon()
    dalt = alt_amsl - home_alt_amsl
    print(f'mc takeoff climb, current altitude: {dalt}')
    if dalt > TARGET_ALTITUDE - 5:
        break  # close enough...

print('Transitioning to fixed-wing...')
px4_gateway.transition_to_fw_sync()
print(f'VTOL state: {px4_gateway.get_vtol_state().name}')
px4_gateway.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
while True:
    time.sleep(1)
    lat, lon, alt_amsl = px4_gateway.get_latlon()
    dalt = alt_amsl - home_alt_amsl
    print(f'fw transition climbout, alt_amsl: {dalt}')
    # wait until we recover to close to our target altitude
    if dalt > TARGET_ALTITUDE - 2:
        break  # close enough...

print('Sending new latlon coordinates...')
new_lat, new_lon = move_relative_meters(home_lat, home_lon, 20, 150)
px4_gateway.go_to_latlon(new_lat, new_lon, home_alt_amsl + TARGET_ALTITUDE)

print('Orbiting new latlon point...')
time.sleep(60)

print('Flying back to orbit launch point...')
px4_gateway.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
for t in range(0, 60):
    time.sleep(1)
    cur_lat, cur_lon, cur_alt_amsl = px4_gateway.get_latlon()
    distance = calc_distance_latlon(cur_lat, cur_lon, home_lat, home_lon)
    print(f't: {t} distance to home: {distance} alt_amsl: {cur_alt_amsl - home_alt_amsl}')
    if distance < 110:
        break

print('Transitioning to multicopter...')
px4_gateway.transition_to_mc_sync()
print(f'VTOL state: {px4_gateway.get_vtol_state().name}')

print('Hover above landing point...')
px4_gateway.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
for t in range(0, 60):
    time.sleep(1)
    cur_lat, cur_lon, cur_alt_amsl = px4_gateway.get_latlon()
    distance = calc_distance_latlon(cur_lat, cur_lon, home_lat, home_lon)
    print(f't: {t} distance to home: {distance} alt_amsl: {cur_alt_amsl - home_alt_amsl}')
    if distance < 1:
        break

print('Descend to low altitude above landing point...')
px4_gateway.go_to_latlon(home_lat, home_lon, home_alt_amsl + 10)
for t in range(0, 30):
    time.sleep(1)
    cur_lat, cur_lon, cur_alt_amsl = px4_gateway.get_latlon()
    rel_alt = cur_alt_amsl - home_alt_amsl
    distance = calc_distance_latlon(cur_lat, cur_lon, home_lat, home_lon)
    print(f't: {t} distance to home: {distance} alt_amsl: {rel_alt}')
    if rel_alt < 12:
        break

print('Landing...')
px4_gateway.land()

# wait during multicopter descent
time.sleep(30)

print('Disarming...')
px4_gateway.disarm_sync()

px4_gateway.destroy()
print('Demo complete.')
