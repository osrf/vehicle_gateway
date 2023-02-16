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

#ifndef VEHICLE_GATEWAY_BETAFLIGHT__VEHICLE_GATEWAY_BETAFLIGHT_HPP_
#define VEHICLE_GATEWAY_BETAFLIGHT__VEHICLE_GATEWAY_BETAFLIGHT_HPP_

#include "vehicle_gateway/vehicle_gateway.hpp"

#include <vector>
#include <memory>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <rclcpp/rclcpp.hpp>

#include <msp/FlightController.hpp>
#include <msp/msp_msg.hpp>

namespace vehicle_gateway_betaflight
{

class VehicleGatewayBetaflight : public vehicle_gateway::VehicleGateway
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
  bool ctbr(float roll, float pitch, float yaw, float throttle) override;

  /// Documentation inherited
  bool set_motors(std::vector<uint16_t> motor_values) override;

private:
  // Orchestration
  std::thread spin_thread_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_raw_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_imu_mag_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_altitude_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_motors_;
  rclcpp::Node::SharedPtr betaflight_node_;

  // Service clients
  vehicle_gateway::ARMING_STATE arming_state_{vehicle_gateway::ARMING_STATE::MAX};

  void onStatus(const msp::msg::Status & status);
  void onBoxNames(const msp::msg::BoxNames & box_names);
  void onImu(const msp::msg::RawImu & imu);
  void onRc(const msp::msg::Rc & rc) {
    // std::cout << rc;
  }
  void onAltitude(const msp::msg::Altitude & altitude);
  void onMotor(const msp::msg::Motor &motor);

  fcu::FlightController fcu_;

  float roll_{0};
  float pitch_{0};
  float yaw_{0};
  float throttle_{0};
  uint16_t arm_{0};

  int index_box_arm_{-1};
};
}  // namespace vehicle_gateway_betaflight

#endif  // VEHICLE_GATEWAY_BETAFLIGHT__VEHICLE_GATEWAY_BETAFLIGHT_HPP_
