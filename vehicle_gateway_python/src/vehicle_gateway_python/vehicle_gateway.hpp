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
    const std::string & _plugin_name);

  ~VehicleGatewayPython();

  /// Takeoff
  void Takeoff();

  /// Land
  void Land();

  /// Arm
  void Arm();

  /// Disarm
  void Disarm();

  /// Transition to multicopter
  void TransitionToMultiCopter();

  /// Transition to fixed wings
  void TransitionToFixedWings();

  void PublishLocalPositionSetpoint(float x, float y, float z);

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

  /// Set offboard_control_mode
  void SetOffboardControlMode(bool is_trajectory);

  /// Set offboard control mode
  void SetOffboardMode();

  /// Get ground speed
  /// \return Get ground speed
  float GetGroundSpeed();

  void FlightTermination(bool termination);

  // Destroy autopilot API
  void Destroy();

private:
  std::shared_ptr<vehicle_gateway::VehicleGateway> gateway_;
  std::shared_ptr<pluginlib::ClassLoader<vehicle_gateway::VehicleGateway>> loader_;
};

void
define_vehicle_gateway(py::object module);

}  // namespace vehicle_gateway_python

#endif  // VEHICLE_GATEWAY_PYTHON__VEHICLE_GATEWAY_HPP_
