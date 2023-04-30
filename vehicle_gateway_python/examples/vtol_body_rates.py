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


vg = vehicle_gateway.init(args=sys.argv, plugin_type='px4')

TARGET_ALTITUDE = 30  # meters above takeoff point

EARTH_RADIUS = 6378100  # meters


def calc_distance_latlon(lat_1, lon_1, lat_2, lon_2):
    # This uses a planar approximation, not the real trigonometry. This
    # approximation is just fine for short distances, which is all we're
    # currently considering.
    dx = (EARTH_RADIUS * math.pi / 180) * (lon_2 - lon_1) * math.cos(lat_1)
    dy = (EARTH_RADIUS * math.pi / 180) * (lat_2 - lat_1)
    return math.sqrt(dx * dx + dy * dy)


def clamp(x, max_magnitude):
    if x > max_magnitude:
        return max_magnitude
    elif x < -max_magnitude:
        return -max_magnitude
    return x


print('Arming...')
vg.arm_sync()
time.sleep(2)  # not sure why... perhaps some internal state setting?

home_lat, home_lon, home_alt_amsl = vg.get_latlon()

print('Takeoff!')
vg.takeoff()

for t in range(0, 10):
    lat, lon, alt_amsl = vg.get_latlon()
    dalt = alt_amsl - home_alt_amsl
    r, p, y = vg.get_euler_rpy()
    print(f'takeoff delay: {dalt} rpy: {r} {p} {y}')
    time.sleep(1)

vg.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
while True:
    time.sleep(1)
    lat, lon, alt_amsl = vg.get_latlon()
    dalt = alt_amsl - home_alt_amsl
    print(f'mc takeoff climb, current altitude: {dalt}')
    if dalt > TARGET_ALTITUDE - 5:
        break  # close enough...

print('Transitioning to fixed-wing...')
vg.transition_to_fw_sync()
print(f'VTOL state: {vg.get_vtol_state().name}')
vg.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
while True:
    time.sleep(1)
    lat, lon, alt_amsl = vg.get_latlon()
    dalt = alt_amsl - home_alt_amsl
    r, p, y = vg.get_euler_rpy()
    print(f'fw transition climbout, alt_amsl: {dalt} rpy: {r} {p} {y}')
    # wait until we recover to close to our target altitude
    if dalt > TARGET_ALTITUDE - 2:
        break  # close enough...

print('begin transitioning to offboard control')
for t in range(0, 20):
    time.sleep(0.1)
    r, p, y = vg.get_euler_rpy()
    print(f'offboard transition {t} / 200, alt_amsl: {dalt} rpy: {r} {p} {y}')
    vg.set_body_rates_and_thrust_setpoint(0, 0, 0, 0.7)
    vg.set_offboard_control_mode(ControllerType.BODY_RATES)

vg.set_offboard_mode()

print('enabled body-rate controller')
target_north = 300
target_east = 0
target_airspeed = 15

lap_count = 0
while True:
    current_ned = vg.get_local_position()
    pos_error = [
        target_north - current_ned[0],
        target_east - current_ned[1],
    ]
    roll, pitch, yaw = vg.get_euler_rpy()
    # yaw comes from PX4 as 0 = north, pi/2 = east
    # let's change that to cartesian with 0 = east, pi / 2 = north
    yaw = math.pi / 2 - yaw
    yaw_target = math.atan2(pos_error[0], pos_error[1])
    yaw_error = yaw_target - yaw
    if yaw_error > math.pi:
        yaw_error -= 2 * math.pi
    elif yaw_error < -math.pi:
        yaw_error += 2 * math.pi

    roll_target = -1.5 * clamp(yaw_error, 0.5)
    roll_error = roll_target - roll
    roll_rate = roll_error

    # minimal altitude controller using pitch rate
    current_altitude = -current_ned[2]
    altitude_error = TARGET_ALTITUDE - current_altitude
    pitch_target = 0.05 * clamp(altitude_error, 5.0)
    pitch_error = pitch_target - pitch
    pitch_rate = 4.0 * pitch_error

    yaw_rate = 0  # without a rudder, it seems yaw_rate doesn't do anything

    # minimal airspeed controller using thrust
    target_airspeed = 20.0
    current_airspeed = vg.get_airspeed()
    airspeed_error = target_airspeed - current_airspeed
    thrust = 0.1 + 0.3 * airspeed_error
    if thrust < 0.1:
        thrust = 0.1
    elif thrust > 1.0:
        thrust = 1.0

    vg.set_offboard_control_mode(ControllerType.BODY_RATES)
    vg.set_body_rates_and_thrust_setpoint(
        roll_rate, pitch_rate, yaw_rate, thrust)
    dist = math.sqrt(pos_error[0] * pos_error[0] + pos_error[1] * pos_error[1])
    print(f'dist: {dist}')

    if dist < 10:
        break
        print('changing target')
        target_north *= -1
        if target_north > 0:
            target_airspeed = 19  # fly fast towards the north
            lap_count += 1
            if lap_count > 1:
                break
        else:
            target_airspeed = 14  # fly slow towards the south

    time.sleep(0.1)

vg.set_onboard_mode()
vg.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
for t in range(0, 60):
    time.sleep(1)
    cur_lat, cur_lon, cur_alt_amsl = vg.get_latlon()
    distance = calc_distance_latlon(cur_lat, cur_lon, home_lat, home_lon)
    print(f't: {t} distance to home: {distance}')
    if distance < 100:
        break

for t in range(0, 60):
    vg.set_airspeed(10)
    time.sleep(0.5)
    current_airspeed = vg.get_airspeed()
    if current_airspeed < 11:
        break
    print(f'slowing down {t} / 10  airspeed: {current_airspeed}')

print('Transitioning to multicopter...')
vg.transition_to_mc_sync()
print(f'VTOL state: {vg.get_vtol_state().name}')

print('Hover above landing point...')
vg.set_onboard_mode()
vg.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
for t in range(0, 60):
    time.sleep(1)
    vg.go_to_latlon(home_lat, home_lon, home_alt_amsl + TARGET_ALTITUDE)
    cur_lat, cur_lon, cur_alt_amsl = vg.get_latlon()
    distance = calc_distance_latlon(cur_lat, cur_lon, home_lat, home_lon)
    print(f't: {t} distance to home: {distance}')
    if distance < 1:
        break

print('Descend to low altitude above landing point...')
vg.go_to_latlon(home_lat, home_lon, home_alt_amsl + 10)
for t in range(0, 30):
    time.sleep(1)
    cur_lat, cur_lon, cur_alt_amsl = vg.get_latlon()
    rel_alt = cur_alt_amsl - home_alt_amsl
    distance = calc_distance_latlon(cur_lat, cur_lon, home_lat, home_lon)
    print(f't: {t} distance to home: {distance} alt_amsl: {rel_alt}')
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
