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
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vtol_vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>

namespace vehicle_gateway_px4
{

class VehicleGatewayPX4 : public vehicle_gateway::VehicleGateway
{
public:
  // Destructor
  ~VehicleGatewayPX4();

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
  vehicle_gateway::VTOL_STATE get_vtol_state() override;

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
  void go_to_latlon(double lat, double lon, float alt_amsl) override;

  /// Documentation inherited
  void set_local_position_setpoint(float x, float y, float z, float yaw) override;

  /// Documentation inherited
  void set_ground_speed(float speed) override;

  /// Documentation inherited
  void set_air_speed(float speed) override;

  /// Documentation inherited
  void set_offboard_control_mode(vehicle_gateway::CONTROLLER_TYPE type) override;

  /// Documentation inherited
  void set_local_velocity_setpoint(float vx, float vy, float vz, float yaw_rate = 0.0f) override;

  /// Documentation inherited
  void set_offboard_mode() override;

  /// Documentation inherited
  void set_onboard_mode() override;

  /// Documentation inherited
  float get_ground_speed() override;

  /// Documentation inherited
  float get_air_speed() override;

  /// Documentation inherited
  float get_altitude() override;

  /// Documentation inherited
  void get_local_position(float & x, float & y, float & z) override;

  /// Documentation inherited
  std::vector<double> get_latlon() override;

  /// Documentation inherited
  bool ctbr(float roll, float pitch, float yaw, float throttle) override;

  /// Documentation inherited
  bool set_motors(std::vector<uint16_t> motor_values) override;

private:
  /// Send command to PX4
  /// \param[in] command Command ID
  /// \param[in] target_system System which should execute the command
  /// \param[in] target_component Component which should execute the command, 0 for all components
  /// \param[in] source_system System sending the command
  /// \param[in] source_component Component sending the command
  /// \param[in] confirmation 0: First transmission of this command
  /// 1-255: Confirmation transmissions
  /// \param[in] param1 Parameter 1, as defined by MAVLink uint16 VEHICLE_CMD enum.
  /// \param[in] param2 Parameter 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
  /// \param[in] param3 Parameter 3, as defined by MAVLink uint16 VEHICLE_CMD enum.
  /// \param[in] param4 Parameter 4, as defined by MAVLink uint16 VEHICLE_CMD enum.
  /// \param[in] param5 Parameter 5, as defined by MAVLink uint16 VEHICLE_CMD enum.
  /// \param[in] param6 Parameter 6, as defined by MAVLink uint16 VEHICLE_CMD enum.
  /// \param[in] param7 Parameter 7, as defined by MAVLink uint16 VEHICLE_CMD enum.
  void send_command(
    uint32_t command, uint8_t target_system, uint8_t target_component, uint8_t source_system,
    uint8_t source_component, uint8_t confirmation, bool from_external,
    float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f,
    float param4 = 0.0f, float param5 = 0.0f, float param6 = 0.0f,
    float param7 = 0.0f);

  /// Send a command to PX4 to set the speed
  /// \param[in] speed Speed to set in m/s
  /// \param[in] is_ground_speed True if the speed is a ground speed, false if it is an air speed
  void set_speed(float speed, bool is_ground_speed);

  // Orchestration
  std::thread spin_thread_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VtolVehicleStatus>::SharedPtr vtol_vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr vehicle_timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr vehicle_sensor_gps_sub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr vehicle_trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    vehicle_offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr
    vehicle_rates_setpoint_pub_;

  // Service clients
  rclcpp::Node::SharedPtr px4_node_;
  vehicle_gateway::ARMING_STATE arming_state_{vehicle_gateway::ARMING_STATE::MAX};
  vehicle_gateway::FLIGHT_MODE flight_mode_{vehicle_gateway::FLIGHT_MODE::UNKNOWN_MODE};
  vehicle_gateway::VEHICLE_TYPE vehicle_type_{vehicle_gateway::VEHICLE_TYPE::UNKNOWN};
  vehicle_gateway::ARM_DISARM_REASON arm_reason_;
  vehicle_gateway::ARM_DISARM_REASON disarm_reason_;
  vehicle_gateway::FAILURE failure_;
  vehicle_gateway::VTOL_STATE vtol_state_{vehicle_gateway::VTOL_STATE::UNDEFINED};

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

  uint8_t target_component_{1};
  uint8_t source_system_{255};
  uint8_t source_component_{0};
  uint8_t confirmation_{1};
  bool from_external_{true};

  std::chrono::time_point<std::chrono::high_resolution_clock> timestamp_;
};

}  // namespace vehicle_gateway_px4

#endif  // VEHICLE_GATEWAY_PX4__VEHICLE_GATEWAY_PX4_HPP_
