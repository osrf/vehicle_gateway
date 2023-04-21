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


px4_gateway = vehicle_gateway.init(args=sys.argv, plugin_type='px4')

TARGET_ALTITUDE = 30  # meters above takeoff point

EARTH_RADIUS = 6378100  # meters


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

print('begin transitioning to offboard control')
for t in range(0, 200):
    time.sleep(0.1)
    px4_gateway.set_local_velocity_setpoint(1, 0, 0, 0)
    px4_gateway.set_offboard_control_mode(ControllerType.VELOCITY)

px4_gateway.set_offboard_mode()

# print('enabled position controller')
print('enabled velocity controller')
target_north = 300
target_east = 0
target_airspeed = 15

lap_count = 0
while True:
    current_ned = px4_gateway.get_local_position()
    vel_cmd = [
        target_north - current_ned[0],
        target_east - current_ned[1],
    ]
    px4_gateway.set_offboard_control_mode(ControllerType.VELOCITY)
    px4_gateway.set_local_velocity_setpoint(vel_cmd[0], vel_cmd[1], 0, 0)
    dist = math.sqrt(vel_cmd[0] * vel_cmd[0] + vel_cmd[1] * vel_cmd[1])
    print(f'dist: {dist}')

    if dist < 10:
        print('changing target')
        target_north *= -1
        if target_north > 0:
            target_airspeed = 19  # fly fast towards the north
            lap_count += 1
            if lap_count > 1:
                break
        else:
            target_airspeed = 14  # fly slow towards the south

    # I don't know why you have to repeatedly send this, but it seems necessary
    px4_gateway.set_air_speed(target_airspeed)

    time.sleep(0.1)

# # (MQ) currently this does not work; for some reason the HOLD controller
# # doesn't really seem to be activated and the vehicle just flies in slow
# # wandering kind-of-circle/spiral, like there is some controller not yet
# # activated to direct heading or something.
# print('Flying back to orbit launch point...')
# time.sleep(0.1)
# px4_gateway.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
# time.sleep(0.1)
# px4_gateway.set_onboard_mode()
# time.sleep(0.1)
# px4_gateway.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
# for t in range(0, 60):
#     time.sleep(1)
#     px4_gateway.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
#     cur_lat, cur_lon, cur_alt_amsl = px4_gateway.get_latlon()
#     distance = calc_distance_latlon(cur_lat, cur_lon, home_lat, home_lon)
#     print(f't: {t} distance to home: {distance} alt_amsl: {cur_alt_amsl - home_alt_amsl}')
#     if distance < 110:
#         break

# fly back towards the launch point at (0, 0)
while True:
    current_ned = px4_gateway.get_local_position()
    vel_cmd = [-current_ned[0], -current_ned[1]]
    px4_gateway.set_offboard_control_mode(ControllerType.VELOCITY)
    px4_gateway.set_local_velocity_setpoint(vel_cmd[0], vel_cmd[1], 0, 0)
    dist = math.sqrt(vel_cmd[0] * vel_cmd[0] + vel_cmd[1] * vel_cmd[1])
    print(f'dist to home: {dist}')
    if dist < 20:
        break
    # I don't know why you have to repeatedly send this, but it seems necessary
    px4_gateway.set_air_speed(14)
    time.sleep(0.1)

print('Transitioning to multicopter...')
px4_gateway.transition_to_mc_sync()
print(f'VTOL state: {px4_gateway.get_vtol_state().name}')

print('Hover above landing point...')
px4_gateway.set_onboard_mode()
px4_gateway.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
for t in range(0, 60):
    time.sleep(1)
    px4_gateway.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
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

# TODO(anyone): it would be nicer here to watch altitude or (even better)
# the descent rate, and break when it's ~zero
time.sleep(30)

print('Disarming...')
px4_gateway.disarm_sync()

px4_gateway.destroy()
print('Demo complete.')
