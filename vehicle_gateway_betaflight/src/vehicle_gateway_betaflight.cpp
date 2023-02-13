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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace vehicle_gateway_betaflight
{

void VehicleGatewayBetaflight::init(int argc, const char ** argv)
{
  rclcpp::init(argc, argv);
  this->exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->betaflight_node_ = std::make_shared<rclcpp::Node>("VehicleGatewayBetaflight");
  this->exec_->add_node(this->betaflight_node_);
  this->spin_thread_ = std::thread(
    [this]() {
      this->exec_->spin();
    });

  this->pub_imu_raw =
    this->betaflight_node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu/data_raw",
    rclcpp::SensorDataQoS());

  std::string device = "/dev/ttyS0";

  this->betaflight_node_->declare_parameter<std::string>("device", device);
  if (!this->betaflight_node_->get_parameter(
      "device", device))
  {
    RCLCPP_WARN_STREAM(
      this->betaflight_node_->get_logger(),
      "The 'device' parameter was not defined, defaulting to: " <<
        device);
  }
  else
  {
    RCLCPP_INFO_STREAM(
      this->betaflight_node_->get_logger(),
      "The 'device' parameter is setted to: " <<
        device);
  }

  int baudrate = 115200;
  this->betaflight_node_->declare_parameter<int>("baudrate", baudrate);
  if (!this->betaflight_node_->get_parameter(
      "baudrate", baudrate))
  {
    RCLCPP_WARN_STREAM(
      this->betaflight_node_->get_logger(),
      "The 'baudrate' parameter was not defined, defaulting to: " <<
        baudrate);
  }
  else
  {
    RCLCPP_INFO_STREAM(
      this->betaflight_node_->get_logger(),
      "The 'baudrate' parameter is setted to: " <<
        baudrate);
  }
  this->fcu_.setLoggingLevel(msp::client::LoggingLevel::INFO);

  // wait for connection
  this->fcu_.connect(device, baudrate, 0, true);

  this->fcu_.setControlSource(fcu::ControlSource::MSP);

  this->fcu_.subscribe(&VehicleGatewayBetaflight::onStatus, this, 1);
  this->fcu_.subscribe(&VehicleGatewayBetaflight::onBoxNames, this, 1);
  this->fcu_.subscribe(&VehicleGatewayBetaflight::onImu, this, 1);
  this->fcu_.subscribe(&VehicleGatewayBetaflight::onRc, this, 1);
}

void VehicleGatewayBetaflight::destroy()
{
  if (this->exec_) {
    this->exec_->cancel();
    rclcpp::shutdown();
    this->spin_thread_.join();
  }
}

void VehicleGatewayBetaflight::onImu(const msp::msg::RawImu & imu)
{
  const msp::msg::ImuSI imu_si(imu, 512.0, 1.0 / 4.096, 0.092, 9.80665);

  std_msgs::msg::Header hdr;
  hdr.stamp = this->betaflight_node_->get_clock()->now();
  hdr.frame_id = "base_link";

  // raw imu data without orientation
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header = hdr;
  imu_msg.linear_acceleration.x = imu_si.acc[0];
  imu_msg.linear_acceleration.y = imu_si.acc[1];
  imu_msg.linear_acceleration.z = imu_si.acc[2];
  imu_msg.angular_velocity.x = imu_si.gyro[0] / 180.0 * M_PI;
  imu_msg.angular_velocity.y = imu_si.gyro[1] / 180.0 * M_PI;
  imu_msg.angular_velocity.z = imu_si.gyro[2] / 180.0 * M_PI;
  this->pub_imu_raw->publish(imu_msg);
}

void VehicleGatewayBetaflight::onBoxNames(const msp::msg::BoxNames & box_names)
{
  if (index_box_arm_ != -1) {
    return;
  }

  std::vector box_names_vector = box_names.box_names;
  auto it = std::find(box_names_vector.begin(), box_names_vector.end(), "ARM");
  // If element was found
  if (it != box_names_vector.end()) {
    // calculating the index
    this->index_box_arm_ = it - box_names_vector.begin();
  }
}

void VehicleGatewayBetaflight::onStatus(const msp::msg::Status & status)
{
  if (this->index_box_arm_ == -1) {
    this->arming_state_ = vehicle_gateway::ARMING_STATE::MAX;
    return;
  }

  auto box_mode_flags = status.box_mode_flags;
  auto it = std::find(box_mode_flags.begin(), box_mode_flags.end(), this->index_box_arm_);
  if (it != box_mode_flags.end()) {
    this->arming_state_ = vehicle_gateway::ARMING_STATE::ARMED;
  } else {
    this->arming_state_ = vehicle_gateway::ARMING_STATE::STANDBY;
  }
}

vehicle_gateway::ARMING_STATE VehicleGatewayBetaflight::get_arming_state()
{
  return this->arming_state_;
}

void VehicleGatewayBetaflight::arm()
{
  this->arm_ = 1900;
}

void VehicleGatewayBetaflight::disarm()
{
  this->arm_ = 1000;
}


bool VehicleGatewayBetaflight::set_motors(std::vector<uint16_t> motor_values)
{
  std::array<uint16_t, msp::msg::N_MOTOR> motors_cmds;
  std::copy_n(motor_values.begin(), msp::msg::N_MOTOR, motors_cmds.begin());
  return this->fcu_.setMotors(motors_cmds);
}

bool VehicleGatewayBetaflight::ctbr(float roll, float pitch, float yaw, float throttle)
{
  this->roll_ = roll;
  this->pitch_ = pitch;
  this->yaw_ = yaw;
  this->throttle_ = throttle;
  std::vector<uint16_t> cmds(6, 1000);
  cmds[0] = uint16_t(this->throttle_ * 500) + 1500;
  cmds[1] = uint16_t(this->yaw_ * 500) + 1500;
  cmds[2] = uint16_t(this->roll_ * 500) + 1500;
  cmds[3] = uint16_t(this->pitch_ * 500) + 1500;

  cmds[4] = this->arm_;
  cmds[5] = 1234;

  for (const auto & c: cmds) {
    std::cerr << c << '\t';
  }
  std::cerr << '\t';
  auto control_source = this->fcu_.getControlSource();
  if (control_source == fcu::ControlSource::MSP) {
    std::cerr << "MSP" << '\n';
  }
  if (control_source == fcu::ControlSource::SBUS) {
    std::cerr << "SBUS" << '\n';
  }
  if (control_source == fcu::ControlSource::NONE) {
    std::cerr << "NONE" << '\n';
  }
  if (control_source == fcu::ControlSource::OTHER) {
    std::cerr << "OTHER" << '\n';
  }

  return this->fcu_.setRc(cmds);
}

vehicle_gateway::FLIGHT_MODE VehicleGatewayBetaflight::get_flight_mode() {}

vehicle_gateway::VEHICLE_TYPE VehicleGatewayBetaflight::get_vehicle_type() {}

vehicle_gateway::ARM_DISARM_REASON VehicleGatewayBetaflight::get_arm_reason() {}

vehicle_gateway::ARM_DISARM_REASON VehicleGatewayBetaflight::get_disarm_reason() {}

vehicle_gateway::FAILURE VehicleGatewayBetaflight::get_failure() {}

void VehicleGatewayBetaflight::takeoff() {}

void VehicleGatewayBetaflight::land() {}

void VehicleGatewayBetaflight::go_to_waypoint() {}

void VehicleGatewayBetaflight::transition_to_fw() {}

void VehicleGatewayBetaflight::transition_to_mc() {}

void VehicleGatewayBetaflight::set_local_position_setpoint(float x, float y, float z) {}

void VehicleGatewayBetaflight::set_offboard_control_mode(bool is_trajectory) {}

void VehicleGatewayBetaflight::set_offboard_mode() {}

float VehicleGatewayBetaflight::get_ground_speed() {}
}  // namespace vehicle_gateway_betaflight
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  vehicle_gateway_betaflight::VehicleGatewayBetaflight,
  vehicle_gateway::VehicleGateway)
