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
from vehicle_gateway import ControllerType


def calc_distance_latlon(lat_1, lon_1, lat_2, lon_2):
    # This uses a planar approximation, not the real trigonometry. This
    # approximation is just fine for short distances, which is all we're
    # currently considering.
    EARTH_RADIUS = 6378100  # meters
    dx = (EARTH_RADIUS * math.pi / 180) * (lon_2 - lon_1) * math.cos(lat_1)
    dy = (EARTH_RADIUS * math.pi / 180) * (lat_2 - lat_1)
    return math.sqrt(dx * dx + dy * dy)


vg = vehicle_gateway.init(args=sys.argv, plugin_type='px4')
TARGET_ALTITUDE = 30  # meters above takeoff point

print('Arming...')
vg.arm_sync()
time.sleep(2)  # not sure why... perhaps some internal state setting?

print('Takeoff!')
home_lat, home_lon, home_alt = vg.get_latlon()
print(f'home_alt: {home_alt}')

vg.takeoff()
time.sleep(1)
vg.go_to_latlon(home_lat, home_lon, home_alt + TARGET_ALTITUDE)
while True:
    time.sleep(1)
    lat, lon, alt = vg.get_latlon()
    dalt = alt - home_alt
    print(f'takeoff climb, current altitude diff: {dalt}')
    if dalt > TARGET_ALTITUDE - 5:
        break  # close enough...

print('Transitioning to fixed-wing...')
vg.transition_to_fw_sync()
print(f'VTOL state: {vg.get_vtol_state().name}')

vg.go_to_latlon(home_lat, home_lon, home_alt + TARGET_ALTITUDE)
while True:
    time.sleep(1)
    lat, lon, alt = vg.get_latlon()
    dalt = alt - home_alt
    print(f'fw transition climbout, alt: {dalt}')
    # wait until we recover to close to our target altitude
    if dalt > TARGET_ALTITUDE - 2:
        break  # close enough...

print('begin transitioning to offboard control')
for t in range(0, 20):
    time.sleep(0.1)
    vg.set_local_position_setpoint(0, 0, -TARGET_ALTITUDE)
    vg.set_offboard_control_mode(ControllerType.POSITION)

vg.set_offboard_mode()

print('enabled position controller')
target_north = 200
target_east = 0
target_airspeed = 15

lap_count = 0
while True:
    current_ned = vg.get_local_position()
    dx = target_east - current_ned[1]
    dy = target_north - current_ned[0]

    vg.set_offboard_control_mode(ControllerType.POSITION)
    vg.set_local_position_setpoint(target_north, target_east, -TARGET_ALTITUDE)
    dist = math.sqrt(dx * dx + dy * dy)
    print(f'dist: {dist}')

    if dist < 10:
        print('changing target')
        target_north *= -1
        if target_north > 0:
            target_airspeed = 15  # fly slow towards the north
            lap_count += 1
            if lap_count >= 1:
                break
        else:
            target_airspeed = 20  # fly fast towards the south

    # I don't know why you have to repeatedly send it, but it seems necessary
    vg.set_airspeed(target_airspeed)
    time.sleep(0.1)

while True:
    vg.set_offboard_control_mode(ControllerType.POSITION)
    vg.set_local_position_setpoint(0, 0, -TARGET_ALTITUDE)
    time.sleep(0.1)
    current_ned = vg.get_local_position()
    dx, dy = -current_ned[1], -current_ned[0]
    dist = math.sqrt(dx * dx + dy * dy)
    print(f'dist: {dist}')
    if dist < 10:
        break

print('Switching back to hold mode...')
vg.set_onboard_mode()
vg.go_to_latlon(home_lat, home_lon, home_alt + TARGET_ALTITUDE)
time.sleep(5)

print('Transitioning to multicopter...')
vg.transition_to_mc_sync()
print(f'VTOL state: {vg.get_vtol_state().name}')

print('Hover above landing point...')
vg.set_onboard_mode()
vg.go_to_latlon(home_lat, home_lon, home_alt + TARGET_ALTITUDE)
for t in range(0, 60):
    time.sleep(1)
    vg.go_to_latlon(home_lat, home_lon, home_alt + TARGET_ALTITUDE)
    cur_lat, cur_lon, cur_alt = vg.get_latlon()
    distance = calc_distance_latlon(cur_lat, cur_lon, home_lat, home_lon)
    print(f't: {t} distance to home: {distance} alt: {cur_alt - home_alt}')
    if distance < 1:
        break

print('Descend to low altitude above landing point...')
vg.go_to_latlon(home_lat, home_lon, home_alt + 10)
for t in range(0, 30):
    time.sleep(1)
    cur_lat, cur_lon, cur_alt = vg.get_latlon()
    rel_alt = cur_alt - home_alt
    distance = calc_distance_latlon(cur_lat, cur_lon, home_lat, home_lon)
    print(f't: {t} distance to home: {distance} alt: {rel_alt}')
    if rel_alt < 12:
        break

print('Landing...')
vg.land()

# TODO(anyone): it would be nicer here to watch altitude or (even better)
# the descent rate, and break when it's ~zero
time.sleep(30)

print('Disarming...')
vg.disarm_sync()

vg.destroy()
print('Demo complete.')
