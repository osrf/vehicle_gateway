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

#include "vehicle_gateway/vehicle_gateway.hpp"
#include <cstdio>

namespace vehicle_gateway_px4
{

class VehicleGatewayPX4 : public vehicle_gateway::VehicleGateway
{
public:
  void init() override
  {
    printf("VehicleGatewayPX4::init()\n");
  }
};

}  // namespace vehicle_gateway_px4

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vehicle_gateway_px4::VehicleGatewayPX4, vehicle_gateway::VehicleGateway)
