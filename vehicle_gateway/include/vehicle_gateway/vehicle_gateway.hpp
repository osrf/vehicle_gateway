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

  virtual void init(int argc, const char ** argv) = 0;

  virtual void destroy() = 0;

  // TODO(anyone): add classes for all the fun cool vehicle things
  virtual void arm() = 0;

  virtual void disarm() = 0;

  virtual bool arming_state() = 0;

  virtual void takeoff() = 0;

  virtual void land() = 0;

  virtual void go_to_waypoint() = 0;

  // VTOL
  virtual void transition_to_fw() = 0;

  virtual void transition_to_mc() = 0;

  virtual void publish_local_position_setpoint(float x, float y, float z) = 0;

  virtual void set_offboard_control_mode(bool is_trajectory) = 0;

  virtual void set_offboard_mode() = 0;

  virtual float get_ground_speed() = 0;

protected:
  VehicleGateway() {}
};
}  // namespace vehicle_gateway

#endif  // VEHICLE_GATEWAY__VEHICLE_GATEWAY_HPP_
