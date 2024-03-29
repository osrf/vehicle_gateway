// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef VEHICLE_GATEWAY_PYTHON__VEHICLE_GATEWAY_HPP_
#define VEHICLE_GATEWAY_PYTHON__VEHICLE_GATEWAY_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <string>
#include <vector>

#include "destroyable.hpp"

#include <pluginlib/class_loader.hpp>
#include <vehicle_gateway/vehicle_gateway.hpp>
#include <vehicle_gateway_px4/vehicle_gateway_px4.hpp>

namespace py = pybind11;

namespace vehicle_gateway_python
{
class VehicleGatewayPython : public Destroyable,
  public std::enable_shared_from_this<VehicleGatewayPython>
{
public:
  VehicleGatewayPython(
    const std::vector<std::string> & _args,
    const std::string & _plugin_name,
    unsigned int _vehicle_id);

  ~VehicleGatewayPython();

  /// Get VehicleID
  unsigned int GetVehicleID();

  /// Takeoff
  void Takeoff();

  /// Land
  void Land();

  /// Arm
  void Arm();

  /// Arm synchronously
  void ArmSync();

  /// Disarm
  void Disarm();

  /// Disarm synchronously
  void DisarmSync();

  /// Transition to multicopter
  void TransitionToMultiCopter();

  /// Transition to multicopter synchronously
  void TransitionToMultiCopterSync();

  /// Transition to fixed wings
  void TransitionToFixedWings();

  /// Transition to fixed wings synchronously
  void TransitionToFixedWingsSync();

  /// Go to latitude and longitude coordinates
  void PublishLatLonSetpoint(double lat, double lon, float alt_amsl);

  /// Get latitude, longitude, and altitude
  std::vector<double> GetLatLon();

  void PublishLocalPositionSetpoint(float x, float y, float z, float yaw);

  void PublishLocalVelocitySetpoint(float vx, float vy, float vz, float yaw_rate);

  void PublishBodyRatesAndThrustSetpoint(
    float roll_rate, float pitch_rate, float yaw_rate,
    float thrust);

  /// Get flight mode
  /// \return Flight mode
  vehicle_gateway::FLIGHT_MODE GetFlightMode();

  /// Get Vehicle type
  /// \return Vehicle type
  vehicle_gateway::VEHICLE_TYPE GetVehicleType();

  /// Get the arm state
  /// \return Arming state of the robot
  vehicle_gateway::ARMING_STATE GetArmingState();

  /// Get the arm reason
  /// \return Arm reason
  vehicle_gateway::ARM_DISARM_REASON GetArmReason();

  /// Get the disarm reason
  /// \return Disarm reason
  vehicle_gateway::ARM_DISARM_REASON GetDisarmReason();

  /// Get current failure
  /// \return Current failure
  vehicle_gateway::FAILURE GetFailure();

  /// Get local position
  /// \return Local position vector
  std::vector<float> GetLocalPosition();

  /// Get Euler attitude
  /// \return vector of euler RPY
  std::vector<float> GetEulerRPY();

  /// Get vtol state
  /// \return VTOL state
  vehicle_gateway::VTOL_STATE GetVtolState();

  /// Set offboard_control_mode
  void SetOffboardControlMode(vehicle_gateway::CONTROLLER_TYPE type);

  /// Set offboard control mode
  void SetOffboardMode();

  /// Set onboard control mode
  void SetOnboardMode();

  /// Get ground speed
  /// \return Get ground speed
  float GetGroundSpeed();

  /// Set ground speed speed m/s
  /// \param[in] Desired speed in m/s
  void SetGroundSpeed(float speed);

  /// Get air speed
  /// \return Get air speed
  float GetAirSpeed();

  /// Set air speed speed m/s
  /// \param[in] Desired air in m/s
  void SetAirSpeed(float speed);

  // Destroy autopilot API
  void Destroy();

  float GetAltitude();

private:
  std::shared_ptr<vehicle_gateway::VehicleGateway> gateway_;
  std::shared_ptr<pluginlib::ClassLoader<vehicle_gateway::VehicleGateway>> loader_;
};

void
define_vehicle_gateway(py::object module);

}  // namespace vehicle_gateway_python

#endif  // VEHICLE_GATEWAY_PYTHON__VEHICLE_GATEWAY_HPP_
