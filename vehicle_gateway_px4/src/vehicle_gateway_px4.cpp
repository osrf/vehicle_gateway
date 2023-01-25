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

#include "vehicle_gateway_px4/vehicle_gateway_px4.hpp"

#include <chrono>
#include <cmath>

#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>

namespace vehicle_gateway_px4
{
void VehicleGatewayPX4::init(int argc, const char ** argv)
{
  rclcpp::init(argc, argv);
  this->exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->px4_node_ = std::make_shared<rclcpp::Node>("VehicleGatewayPX4");
  this->exec_->add_node(px4_node_);
  this->spin_thread_ = std::thread(
    [this]() {
      this->exec_->spin();
    });

  rclcpp::QoS qos_profile(10);
  qos_profile
  // Guaranteed delivery is needed to send messages to late-joining subscriptions.
  .best_effort();

  this->vehicle_status_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status",
    qos_profile,
    [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
      this->arming_state_ = msg->arming_state;
      this->nav_state_ = msg->nav_state;
    });

  this->vehicle_sensor_gps_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::SensorGps>(
    "/fmu/out/vehicle_gps_position",
    qos_profile,
    [this](px4_msgs::msg::SensorGps::ConstSharedPtr msg) {
      this->lat_ = msg->lat;
      this->lon_ = msg->lon;
      this->alt_ = msg->alt;
    });

  this->vehicle_timesync_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::TimesyncStatus>(
    "/fmu/out/timesync_status",
    qos_profile,
    [this](px4_msgs::msg::TimesyncStatus::ConstSharedPtr msg) {
      this->timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
        std::chrono::nanoseconds(msg->timestamp));
  });

  this->vehicle_odometry_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry",
    qos_profile,
    [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
      this->odom_timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
        std::chrono::nanoseconds(msg->timestamp));
      this->ground_speed_ = std::sqrt(std::pow(msg->velocity[0], 2) + std::pow(msg->velocity[1], 2));

      current_pos_x_ = msg->position[0];
      current_pos_y_ = msg->position[1];
      current_pos_z_ = msg->position[2];
      current_vel_x_ = msg->velocity[0];
      current_vel_y_ = msg->velocity[1];
      current_vel_z_ = msg->velocity[2];
  });

  this->vehicle_command_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", qos_profile);
  this->vehicle_trajectory_setpoint_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", qos_profile);
  this->vehicle_offboard_control_mode_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", qos_profile);
}

void VehicleGatewayPX4::destroy()
{
  if (this->exec_) {
    this->exec_->cancel();
    rclcpp::shutdown();
    this->spin_thread_.join();
  }
}

bool VehicleGatewayPX4::arming_state()
{
  return arming_state_ == 2;
}

void VehicleGatewayPX4::arm()
{
  px4_msgs::msg::VehicleCommand msg_vehicle_command;

  msg_vehicle_command.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000.0;
  msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  msg_vehicle_command.param1 = 1;
  msg_vehicle_command.confirmation = 1;
  msg_vehicle_command.source_system = 255;
  msg_vehicle_command.target_system = this->target_system_;
  msg_vehicle_command.target_component = 1;
  msg_vehicle_command.from_external = true;
  this->vehicle_command_pub_->publish(msg_vehicle_command);
}

void VehicleGatewayPX4::disarm()
{
  px4_msgs::msg::VehicleCommand msg_vehicle_command;

  msg_vehicle_command.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000.0;
  msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  msg_vehicle_command.param1 = 0;
  msg_vehicle_command.confirmation = 1;
  msg_vehicle_command.source_system = 255;
  msg_vehicle_command.target_system = this->target_system_;
  msg_vehicle_command.target_component = 1;
  msg_vehicle_command.from_external = true;
  this->vehicle_command_pub_->publish(msg_vehicle_command);
}

void VehicleGatewayPX4::set_offboard_mode()
{
  px4_msgs::msg::VehicleCommand msg_vehicle_command;

  msg_vehicle_command.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;
  msg_vehicle_command.param1 = 1;
  msg_vehicle_command.param2 = 6;
  msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;  // command ID
  msg_vehicle_command.target_system = this->target_system_;  // system which should execute the command
  msg_vehicle_command.target_component = 1;  // component to execute the command, 0 for all
  msg_vehicle_command.source_system = 255;  // system sending the command
  msg_vehicle_command.source_component = 1;  // component sending the command
  msg_vehicle_command.from_external = true;
  this->vehicle_command_pub_->publish(msg_vehicle_command);
}

float VehicleGatewayPX4::get_ground_speed()
{
  return this->ground_speed_;
}

void VehicleGatewayPX4::takeoff()
{
  px4_msgs::msg::VehicleCommand msg_vehicle_command;

  // take off
  msg_vehicle_command.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;
  msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  msg_vehicle_command.param1 = 0.1;
  msg_vehicle_command.param2 = 0;
  msg_vehicle_command.param3 = 0;
  msg_vehicle_command.param4 = 1.57;  // orientation
  msg_vehicle_command.param5 = this->lat_ * 1e-7;
  msg_vehicle_command.param6 = this->lon_ * 1e-7;
  msg_vehicle_command.param7 = 5.0;
  msg_vehicle_command.confirmation = 1;
  msg_vehicle_command.source_system = 255;
  msg_vehicle_command.target_system = this->target_system_;
  msg_vehicle_command.target_component = 1;
  msg_vehicle_command.from_external = true;
  this->vehicle_command_pub_->publish(msg_vehicle_command);

  msg_vehicle_command.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;
  msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
  msg_vehicle_command.param1 = -1;
  msg_vehicle_command.param2 = 1;
  msg_vehicle_command.param3 = 0;
  msg_vehicle_command.param4 = 1.57;
  msg_vehicle_command.param5 = this->lat_ * 1e-7;
  msg_vehicle_command.param6 = this->lon_ * 1e-7;
  msg_vehicle_command.param7 = this->alt_ * 1e-3 + 5;
  msg_vehicle_command.confirmation = 0;
  msg_vehicle_command.source_system = 255;
  msg_vehicle_command.target_system = this->target_system_;
  msg_vehicle_command.target_component = 1;
  msg_vehicle_command.from_external = true;
  this->vehicle_command_pub_->publish(msg_vehicle_command);
}

void VehicleGatewayPX4::land()
{
  px4_msgs::msg::VehicleCommand msg_vehicle_command;

  msg_vehicle_command.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;
  msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
  msg_vehicle_command.param1 = 0.1;
  msg_vehicle_command.param4 = 1.57;
  msg_vehicle_command.param5 = this->lat_ * 1e-7;
  msg_vehicle_command.param6 = this->lon_ * 1e-7;
  msg_vehicle_command.confirmation = 1;
  msg_vehicle_command.source_system = 255;
  msg_vehicle_command.target_system = this->target_system_;
  msg_vehicle_command.target_component = 1;
  msg_vehicle_command.from_external = true;
  this->vehicle_command_pub_->publish(msg_vehicle_command);
}

void VehicleGatewayPX4::transition_to_fw()
{
  px4_msgs::msg::VehicleCommand msg_vehicle_command;

  msg_vehicle_command.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;
  msg_vehicle_command.param1 = 4.0;
  msg_vehicle_command.param2 = 1.0;
  msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION;  // command ID
  msg_vehicle_command.target_system = this->target_system_;  // system which should execute the command
  msg_vehicle_command.target_component = 1;  // component to execute the command, 0 for all
  msg_vehicle_command.source_system = 255;  // system sending the command
  msg_vehicle_command.source_component = 1;  // component sending the command
  msg_vehicle_command.from_external = true;
  this->vehicle_command_pub_->publish(msg_vehicle_command);
}

void VehicleGatewayPX4::transition_to_mc()
{
  px4_msgs::msg::VehicleCommand msg_vehicle_command;

  msg_vehicle_command.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;
  msg_vehicle_command.param1 = 3.0;
  msg_vehicle_command.param2 = 1.0;
  msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION;  // command ID
  msg_vehicle_command.target_system = this->target_system_;  // system which should execute the command
  msg_vehicle_command.target_component = 1;  // component to execute the command, 0 for all
  msg_vehicle_command.source_system = 255;  // system sending the command
  msg_vehicle_command.source_component = 1;  // component sending the command
  msg_vehicle_command.from_external = true;
  this->vehicle_command_pub_->publish(msg_vehicle_command);
}

void VehicleGatewayPX4::publish_local_position_setpoint(float x, float y, float z)
{
  px4_msgs::msg::TrajectorySetpoint msg;

  msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

  msg.position[0] = x;
  msg.position[1] = y;
  msg.position[2] = z;
  this->vehicle_trajectory_setpoint_pub_->publish(msg);

  px4_msgs::msg::VehicleCommand msg_vehicle_command;

  msg_vehicle_command.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;
  msg_vehicle_command.param1 = 0;
  msg_vehicle_command.param2 = 1.0;
  msg_vehicle_command.param3 = -1;
  msg_vehicle_command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED;  // command ID
  msg_vehicle_command.target_system = this->target_system_;  // system which should execute the command
  msg_vehicle_command.target_component = 1;  // component to execute the command, 0 for all
  msg_vehicle_command.source_system = 255;  // system sending the command
  msg_vehicle_command.source_component = 1;  // component sending the command
  msg_vehicle_command.from_external = true;
  this->vehicle_command_pub_->publish(msg_vehicle_command);
}

void VehicleGatewayPX4::set_offboard_control_mode(bool is_trajectory)
{
  px4_msgs::msg::OffboardControlMode msg;
  msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

  msg.position = is_trajectory;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = !is_trajectory;

  this->vehicle_offboard_control_mode_pub_->publish(msg);
}

void VehicleGatewayPX4::go_to_waypoint()
{
}
}  // namespace vehicle_gateway_px4
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vehicle_gateway_px4::VehicleGatewayPX4, vehicle_gateway::VehicleGateway)
