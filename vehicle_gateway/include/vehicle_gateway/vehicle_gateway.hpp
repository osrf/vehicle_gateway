// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VEHICLE_GATEWAY__VEHICLE_GATEWAY_HPP_
#define VEHICLE_GATEWAY__VEHICLE_GATEWAY_HPP_

#include <cmath>
#include <cstdint>
#include <vector>

namespace vehicle_gateway
{

enum ARMING_STATE
{
  INIT = 0,
  STANDBY = 1,
  ARMED = 2,
  STANDBY_ERROR = 3,
  SHUTTEDDOWN = 4,
  IN_AIR_RESTORE = 5,
  MAX = 6
};

enum ARM_DISARM_REASON
{
  TRANSITION_TO_STANDBY = 0,
  RC_STICK = 1,
  RC_SWITCH = 2,
  COMMAND_INTERNAL = 3,
  COMMAND_EXTERNAL = 4,
  MISSION_START = 5,
  SAFETY_BUTTON = 6,
  AUTO_DISARM_LAND = 7,
  AUTO_DISARM_PREFLIGHT = 8,
  KILL_SWITCH = 9,
  LOCKDOWN = 10,
  FAILURE_DETECTOR = 11,
  SHUTDOWN = 12,
  ARM_DISARM_REASON_NONE = 13
};

enum FAILURE
{
  NONE = 0,
  ROLL = 1,
  PITCH = 2,
  ALT = 4,
  EXT = 8,
  ARM_ESC = 16,
  BATTERY = 32,
  IMBALANCED_PROP = 64,
  MOTOR = 128
};

enum VEHICLE_TYPE
{
  UNKNOWN = 0,
  ROTARY_WING = 1,
  FIXED_WING = 2,
  ROVER = 3,
  AIRSHIP = 4
};

enum FLIGHT_MODE
{
  MANUAL = 0,               // Manual mode
  ALTCTL = 1,               // Altitude control mode
  POSCTL = 2,               // Position control mode
  AUTO_MISSION = 3,         // Auto mission mode
  AUTO_LOITER = 4,          // Auto loiter mode
  AUTO_RTL = 5,             // Auto return to launch mode
  ACRO = 6,                 // Acro mode
  DESCEND = 7,              // Descend mode (no position control)
  TERMINATION = 8,          // Termination mode
  OFFBOARD = 9,             // Offboard
  STAB = 10,                // Stabilized mode
  AUTO_TAKEOFF = 11,        // Takeoff
  AUTO_LAND = 12,           // Land
  AUTO_FOLLOW_TARGET = 13,  // Auto Follow
  AUTO_PRECLAND = 14,       // Precision land with landing target
  ORBIT = 15,               // Orbit in a circle
  AUTO_VTOL_TAKEOFF = 16,   // Takeoff, transition, establish loiter
  UNKNOWN_MODE = 17
};

enum CONTROLLER_TYPE
{
  NO_CONTROLLER = 0,        // No controller defined
  POSITION = 1,             // Position control
  VELOCITY = 2,             // Velocity control
  BODY_RATES = 3,           // Body rates (rad/s) and thrust [-1, 1] controller
};

enum VTOL_STATE  // Based on https://mavlink.io/en/messages/common.html#MAV_VTOL_STATE
{
  UNDEFINED = 0,            // MAV is not configured as VTOL
  TRANSITION_TO_FW = 1,     // VTOL is in transition from multicopter to fixed-wing
  TRANSITION_TO_MC = 2,     // VTOL is in transition from fixed-wing to multicopter
  MC = 3,                   // VTOL is in multicopter state
  FW = 4,                   // VTOL is in fixed-wing state
};

class VehicleGateway
{
public:
  virtual ~VehicleGateway() {}

  /// Initialize the autopilot
  virtual void init(int argc, const char ** argv) = 0;

  /// Set Vehicle ID
  /// \param[in] _vehicle_id Vehicle ID
  virtual void set_vehicle_id(unsigned int _vehicle_id) = 0;

  /// Get Vehicle ID
  /// \return Vehicle ID
  virtual unsigned int get_vehicle_id() = 0;

  /// Destroy the gateway
  virtual void destroy() = 0;

  /// Arm vehicle
  virtual void arm() = 0;

  /// Arm vehicle (blocking method)
  virtual void arm_sync() = 0;

  /// Disarm vehicle
  virtual void disarm() = 0;

  /// Disarm vehicle (blocking method)
  virtual void disarm_sync() = 0;

  /// Get the arm state
  /// \return Arming state of the robot
  virtual ARMING_STATE get_arming_state() = 0;

  /// Get the arm reason
  /// \return Arm reason
  virtual ARM_DISARM_REASON get_arm_reason() = 0;

  /// Get the disarm reason
  /// \return Disarm reason
  virtual ARM_DISARM_REASON get_disarm_reason() = 0;

  /// Get current failure
  /// \return Current failure
  virtual FAILURE get_failure() = 0;

  /// Get flight mode
  /// \return Flight mode
  virtual FLIGHT_MODE get_flight_mode() = 0;

  /// Get Vehicle type
  /// \return Vehicle type
  virtual VEHICLE_TYPE get_vehicle_type() = 0;

  /// Get VTOL state
  /// \return VTOL state
  virtual VTOL_STATE get_vtol_state() = 0;

  /// Takeoff the robot
  virtual void takeoff() = 0;

  /// Land the robot
  virtual void land() = 0;

  /// Got to waypoint
  virtual void go_to_waypoint() = 0;

  /// VTOL
  /// Transition to fixed wings
  virtual void transition_to_fw() = 0;

  /// Transition to fixed wings (blocking method)
  virtual void transition_to_fw_sync() = 0;

  /// Transition to multicopter
  virtual void transition_to_mc() = 0;

  /// Transition to multicopter (blocking method)
  virtual void transition_to_mc_sync() = 0;

  /// Go to latitude and longitude coordinates
  /// \param[in] lat Desired latitude coordinate
  /// \param[in] lon Desired longitude coordinate
  /// \param[in] alt_amsl Desired altitude above mean sea level (AMSL) in meters
  virtual void go_to_latlon(double lat, double lon, float alt_amsl) = 0;

  /// Go to latitude and longitude coordinates
  /// \param[in] lat Desired latitude coordinate
  /// \param[in] lon Desired longitude coordinate
  /// \param[in] alt_amsl Desired altitude above mean sea level (AMSL) in meters
  virtual void go_to_latlon_sync(
    double lat, double lon, double alt,
    double latlon_threshold = 0.5, double alt_threshold = 0.5) = 0;

  /// Set local position
  /// \param[in] x Desired x position
  /// \param[in] y Desired y position
  /// \param[in] z Desired z position
  /// \param[in] yaw Desired yaw position
  virtual void set_local_position_setpoint(float x, float y, float z, float yaw) = 0;

  /// Set local position
  /// \param[in] x Desired x position
  /// \param[in] y Desired y position
  /// \param[in] z Desired z position
  /// \param[in] yaw Desired yaw position
  virtual void offboard_mode_go_to_local_setpoint_sync(
    double x,
    double y,
    double alt,
    double yaw = std::numeric_limits<float>::quiet_NaN(),
    double airspeeed = 15.0,
    double distance_threshold = 10.0,
    vehicle_gateway::CONTROLLER_TYPE controller_type = vehicle_gateway::CONTROLLER_TYPE::POSITION) =
  0;

  /// Set ground speed in m/s
  /// \param[in] speed Desired speed in m/s
  virtual void set_ground_speed(float speed) = 0;

  /// Set air speed in m/s
  /// \param[in] speed Desired speed in m/s
  virtual void set_airspeed(float speed) = 0;

  /// Set local velocity
  /// \param[in] vx Desired x velocity
  /// \param[in] vy Desired y velocity
  /// \param[in] vz Desired z velocity
  virtual void set_local_velocity_setpoint(float vx, float vy, float vz, float yaw_rate = 0.0f) = 0;

  /// Set controller type
  /// \param[in] type Controller type
  virtual void set_offboard_control_mode(CONTROLLER_TYPE type) = 0;

  /// Set offboard mode (blocking method)
  virtual void transition_to_offboard_sync() = 0;

  /// Set offboard mode
  virtual void set_offboard_mode() = 0;

  /// Set onboard mode (the opposite of the previous function)
  virtual void set_onboard_mode() = 0;

  /// Get ground speed
  /// \return Get ground speed
  virtual float get_ground_speed() = 0;

  /// Get ground speed
  /// \return Get ground speed
  virtual float get_airspeed() = 0;

  /// Get altitude
  /// \return Get altitude
  virtual float get_altitude() = 0;

  /// Get 0: latitude, 1: longitude, and 2: altitude
  virtual std::vector<double> get_latlon() = 0;

  virtual void get_local_position(float & x, float & y, float & z) = 0;

  virtual void get_euler_rpy(float & roll, float & pitch, float & yaw) = 0;

  virtual bool set_body_rates_and_thrust_setpoint(
    float roll_rate, float pitch_rate, float yaw_rate, float thrust) = 0;

  virtual bool set_motors(std::vector<uint16_t> motor_values) = 0;

protected:
  VehicleGateway() {}
};
}  // namespace vehicle_gateway

#endif  // VEHICLE_GATEWAY__VEHICLE_GATEWAY_HPP_
