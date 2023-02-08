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

#include "vehicle_gateway_betaflight/vehicle_gateway_betaflight.hpp"

#include <chrono>
#include <cmath>

namespace vehicle_gateway_betaflight
{

void VehicleGatewayBetaflight::init(int argc, const char ** argv)
{
  const std::string device =
      (argc > 1) ? std::string(argv[1]) : "/dev/ttyS0";
  const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

  fcu.setLoggingLevel(msp::client::LoggingLevel::INFO);
  // wait for connection
  fcu.connect(device, baudrate);

  fcu.subscribe(&VehicleGatewayBetaflight::onStatus, this, 1);
}

void VehicleGatewayBetaflight::destroy()
{

}

void VehicleGatewayBetaflight::onStatus(const msp::msg::Status& status)
{
  std::cout << "status " << status;
}

vehicle_gateway::ARMING_STATE VehicleGatewayBetaflight::get_arming_state()
{
  return this->arming_state_;
}

void VehicleGatewayBetaflight::arm(){}

void VehicleGatewayBetaflight::disarm(){}

vehicle_gateway::FLIGHT_MODE VehicleGatewayBetaflight::get_flight_mode(){}

vehicle_gateway::VEHICLE_TYPE VehicleGatewayBetaflight::get_vehicle_type(){}

vehicle_gateway::ARM_DISARM_REASON VehicleGatewayBetaflight::get_arm_reason(){}

vehicle_gateway::ARM_DISARM_REASON VehicleGatewayBetaflight::get_disarm_reason(){}

vehicle_gateway::FAILURE VehicleGatewayBetaflight::get_failure(){}

void VehicleGatewayBetaflight::takeoff(){}

void VehicleGatewayBetaflight::land(){}

void VehicleGatewayBetaflight::go_to_waypoint(){}

void VehicleGatewayBetaflight::transition_to_fw(){}

void VehicleGatewayBetaflight::transition_to_mc(){}

void VehicleGatewayBetaflight::set_local_position_setpoint(float x, float y, float z){}

void VehicleGatewayBetaflight::set_offboard_control_mode(bool is_trajectory){}

void VehicleGatewayBetaflight::set_offboard_mode(){}

float VehicleGatewayBetaflight::get_ground_speed(){}
}  // namespace vehicle_gateway_betaflight
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vehicle_gateway_betaflight::VehicleGatewayBetaflight, vehicle_gateway::VehicleGateway)
