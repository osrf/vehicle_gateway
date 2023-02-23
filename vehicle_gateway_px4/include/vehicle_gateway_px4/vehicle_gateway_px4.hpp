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

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace vehicle_gateway_px4
{

class VehicleGatewayPX4 : public vehicle_gateway::VehicleGateway
{
public:
  /// Documentation inherited
  void init(int argc, const char ** argv) override;

  /// Documentation inherited
  void destroy() override;

  /// Documentation inherited
  void arm() override;

  /// Documentation inherited
  void disarm() override;

  /// Documentation inherited
  vehicle_gateway::ARMING_STATE get_arming_state() override;

  /// Documentation inherited
  vehicle_gateway::FLIGHT_MODE get_flight_mode() override;

  /// Documentation inherited
  vehicle_gateway::VEHICLE_TYPE get_vehicle_type() override;

  /// Documentation inherited
  vehicle_gateway::ARM_DISARM_REASON get_arm_reason() override;

  /// Documentation inherited
  vehicle_gateway::ARM_DISARM_REASON get_disarm_reason() override;

  /// Documentation inherited
  vehicle_gateway::FAILURE get_failure() override;

  /// Documentation inherited
  void takeoff() override;

  /// Documentation inherited
  void land() override;

  /// Documentation inherited
  void go_to_waypoint() override;

  /// Documentation inherited
  void transition_to_fw() override;

  /// Documentation inherited
  void transition_to_mc() override;

  /// Documentation inherited
  void set_local_position_setpoint(float x, float y, float z) override;

  /// Documentation inherited
  void set_offboard_control_mode(bool is_trajectory) override;

  /// Documentation inherited
  void set_offboard_mode() override;

  /// Documentation inherited
  float get_ground_speed() override;

  /// Documentation inherited
  float get_altitude() override;

private:
  // Orchestration
  std::thread spin_thread_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr vehicle_timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr vehicle_sensor_gps_sub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr vehicle_trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    vehicle_offboard_control_mode_pub_;

  // Service clients
  rclcpp::Node::SharedPtr px4_node_;
  vehicle_gateway::ARMING_STATE arming_state_{vehicle_gateway::ARMING_STATE::MAX};
  vehicle_gateway::FLIGHT_MODE flight_mode_{vehicle_gateway::FLIGHT_MODE::UNKNOWN_MODE};
  vehicle_gateway::VEHICLE_TYPE vehicle_type_{vehicle_gateway::VEHICLE_TYPE::UNKNOWN};
  vehicle_gateway::ARM_DISARM_REASON arm_reason_;
  vehicle_gateway::ARM_DISARM_REASON disarm_reason_;
  vehicle_gateway::FAILURE failure_;

  int target_system_{1};

  double lat_{0};
  double lon_{0};
  float alt_{0};

  float current_speed;
  std::chrono::time_point<std::chrono::high_resolution_clock> odom_timestamp_;
  float current_pos_x_;
  float current_pos_y_;
  float current_pos_z_;
  float current_vel_x_;
  float current_vel_y_;
  float current_vel_z_;
  float ground_speed_;

  std::chrono::time_point<std::chrono::high_resolution_clock> timestamp_;
};

}  // namespace vehicle_gateway_px4

#endif  // VEHICLE_GATEWAY_PX4__VEHICLE_GATEWAY_PX4_HPP_
