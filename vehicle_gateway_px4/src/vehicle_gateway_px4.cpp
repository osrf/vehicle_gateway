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

  this->vehicle_command_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", qos_profile);
}

void VehicleGatewayPX4::destroy()
{
  if (this->exec_)
  {
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
  msg_vehicle_command.target_system = target_system_;
  msg_vehicle_command.target_component = 1;
  msg_vehicle_command.from_external = true;
  this->vehicle_command_pub_->publish(msg_vehicle_command);
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
  msg_vehicle_command.target_system = target_system_;
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
  msg_vehicle_command.target_system = target_system_;
  msg_vehicle_command.target_component = 1;
  msg_vehicle_command.from_external = true;
  vehicle_command_pub_->publish(msg_vehicle_command);
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
  msg_vehicle_command.target_system = target_system_;
  msg_vehicle_command.target_component = 1;
  msg_vehicle_command.from_external = true;
  vehicle_command_pub_->publish(msg_vehicle_command);
}

void VehicleGatewayPX4::go_to_waypoint()
{
}
}  // namespace vehicle_gateway_px4
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vehicle_gateway_px4::VehicleGatewayPX4, vehicle_gateway::VehicleGateway)
