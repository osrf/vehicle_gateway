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

#ifndef VEHICLE_GATEWAY_PX4__VEHICLE_GATEWAY_PX4_HPP_
#define VEHICLE_GATEWAY_PX4__VEHICLE_GATEWAY_PX4_HPP_

#include "vehicle_gateway/vehicle_gateway.hpp"

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

namespace vehicle_gateway_px4
{

class VehicleGatewayPX4 : public vehicle_gateway::VehicleGateway
{
public:
  void init(int argc, const char ** argv) override;

public:
  void destroy() override;

public:
  void arm() override;

public:
  bool arming_state() override;

public:
  void takeoff() override;

public:
  void land() override;

public:
  void go_to_waypoint() override;

private:
  // Orchestration
  std::thread spin_thread_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr vehicle_sensor_gps_sub_;

  // Service clients
  rclcpp::Node::SharedPtr px4_node_;
  int arming_state_;
  int nav_state_;

  int target_system_{1};

  double lat_{0};
  double lon_{0};
  float alt_{0};
};

}  // namespace vehicle_gateway_px4

#endif  // VEHICLE_GATEWAY_PX4__VEHICLE_GATEWAY_PX4_HPP_
