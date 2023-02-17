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
  SHUTDOWN = 12
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

class VehicleGateway
{
public:
  // TODO(anyone): add classes for all the fun cool vehicle things

  virtual ~VehicleGateway() {}

  /// Initialize the autopilot
  virtual void init(int argc, const char ** argv) = 0;

  /// Destroy the gateway
  virtual void destroy() = 0;

  /// Arm vehicle
  virtual void arm() = 0;

  /// Disarm vehicle
  virtual void disarm() = 0;

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

  // Takeoff the robot
  virtual void takeoff() = 0;

  /// Land the robot
  virtual void land() = 0;

  /// Got to waypoint
  virtual void go_to_waypoint() = 0;

  // VTOL
  /// Transition to fixed wings
  virtual void transition_to_fw() = 0;

  /// Transition to multicopter
  virtual void transition_to_mc() = 0;

  /// Set local position
  /// \param[in] x Desired x position
  /// \param[in] y Desired y position
  /// \param[in] z Desired z position
  virtual void set_local_position_setpoint(float x, float y, float z) = 0;

  /// Set offboard_control_mode
  virtual void set_offboard_control_mode(bool is_trajectory) = 0;

  /// Set offboard mode
  virtual void set_offboard_mode() = 0;

  /// Get ground speed
  /// \return Get ground speed
  virtual float get_ground_speed() = 0;

  /// Get altitude
  /// \return Get altitude
  virtual float get_altitude() = 0;

  virtual void get_local_position(float &x, float &y, float &z) = 0;

protected:
  VehicleGateway() {}
};
}  // namespace vehicle_gateway

#endif  // VEHICLE_GATEWAY__VEHICLE_GATEWAY_HPP_
