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

class VehicleGateway
{
public:
  virtual ~VehicleGateway() {}

  virtual void init() = 0;

  // TODO(anyone): add classes for all the fun cool vehicle things

protected:
  VehicleGateway() {}
};
}  // namespace vehicle_gateway

#endif  // VEHICLE_GATEWAY__VEHICLE_GATEWAY_HPP_
