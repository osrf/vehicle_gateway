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
  if (argc != 0 && argv != nullptr) {
    rclcpp::init(argc, argv);
  }
  this->exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->betaflight_node_ = std::make_shared<rclcpp::Node>("VehicleGatewayBetaflight");
  this->exec_->add_node(this->betaflight_node_);
  this->spin_thread_ = std::thread(
    [this]() {
      this->exec_->spin();
    });

  this->pub_imu_raw_ =
    this->betaflight_node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu/data_raw",
    rclcpp::SensorDataQoS());

  this->pub_imu_mag_ =
    this->betaflight_node_->create_publisher<sensor_msgs::msg::MagneticField>(
    "imu/mag",
    rclcpp::SensorDataQoS());

  this->pub_altitude_ =
    this->betaflight_node_->create_publisher<std_msgs::msg::Float64>(
    "global_position/rel_alt",
    rclcpp::SensorDataQoS());

  this->pub_motors_ =
    this->betaflight_node_->create_publisher<std_msgs::msg::UInt16MultiArray>(
    "motors_out",
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
  } else {
    RCLCPP_INFO_STREAM(
      this->betaflight_node_->get_logger(),
      "The 'device' parameter is set to: " <<
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
  } else {
    RCLCPP_INFO_STREAM(
      this->betaflight_node_->get_logger(),
      "The 'baudrate' parameter is set to: " <<
        baudrate);
  }
  this->fcu_.setLoggingLevel(msp::client::LoggingLevel::INFO);

  // wait for connection
  this->fcu_.connect(device, baudrate, 0, true);

  this->fcu_.setControlSource(fcu::ControlSource::MSP);

  this->fcu_.subscribe(&VehicleGatewayBetaflight::onStatus, this, 1);
  this->fcu_.subscribe(&VehicleGatewayBetaflight::onBoxNames, this, 1);
  this->fcu_.subscribe(&VehicleGatewayBetaflight::onImu, this, 1);
  this->fcu_.subscribe(&VehicleGatewayBetaflight::onAltitude, this, 1);
  this->fcu_.subscribe(&VehicleGatewayBetaflight::onMotor, this, 1);
}

void VehicleGatewayBetaflight::destroy()
{
  if (this->exec_) {
    this->exec_->cancel();
    rclcpp::shutdown();
    this->spin_thread_.join();
  }
}

void VehicleGatewayBetaflight::onAltitude(const msp::msg::Altitude & altitude)
{
  // altitude in meter
  std_msgs::msg::Float64 alt;
  alt.data = altitude.altitude;
  this->altitude = altitude.altitude;
  this->pub_altitude_->publish(alt);
}

float VehicleGatewayBetaflight::get_altitude()
{
  return this->altitude;
}

void VehicleGatewayBetaflight::onMotor(const msp::msg::Motor & motor)
{
  std_msgs::msg::UInt16MultiArray motor_out;
  for (const uint16_t m : motor.motor) {
    motor_out.data.push_back(m);
  }
  this->pub_motors_->publish(motor_out);
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
  this->pub_imu_raw_->publish(imu_msg);

  // magnetic field vector
  sensor_msgs::msg::MagneticField mag_msg;
  mag_msg.header = hdr;
  mag_msg.magnetic_field.x = imu_si.mag[0] * 1e-6;
  mag_msg.magnetic_field.y = imu_si.mag[1] * 1e-6;
  mag_msg.magnetic_field.z = imu_si.mag[2] * 1e-6;
  this->pub_imu_mag_->publish(mag_msg);
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
  cmds[2] = uint16_t(this->throttle_ * 500) + 1500;
  cmds[3] = uint16_t(this->yaw_ * 500) + 1500;
  cmds[1] = uint16_t(this->roll_ * 500) + 1500;
  cmds[0] = uint16_t(this->pitch_ * 500) + 1500;

  cmds[4] = this->arm_;

  return this->fcu_.setRc(cmds);
}

vehicle_gateway::FLIGHT_MODE VehicleGatewayBetaflight::get_flight_mode()
{
  return vehicle_gateway::FLIGHT_MODE::MANUAL;
}

vehicle_gateway::VEHICLE_TYPE VehicleGatewayBetaflight::get_vehicle_type()
{
  return vehicle_gateway::VEHICLE_TYPE::ROTARY_WING;
}

vehicle_gateway::ARM_DISARM_REASON VehicleGatewayBetaflight::get_arm_reason()
{
  // TODO(anyone): update with correct reason
  return vehicle_gateway::ARM_DISARM_REASON_NONE;
}

vehicle_gateway::ARM_DISARM_REASON VehicleGatewayBetaflight::get_disarm_reason()
{
  // TODO(anyone): update with correct reason
  return vehicle_gateway::ARM_DISARM_REASON_NONE;
}

vehicle_gateway::FAILURE VehicleGatewayBetaflight::get_failure()
{
  // TODO(anyone): update with correct reason
  return vehicle_gateway::NONE;
}

vehicle_gateway::VTOL_STATE VehicleGatewayBetaflight::get_vtol_state()
{
  // TODO(anyone): update with correct state
  return vehicle_gateway::VTOL_STATE::UNDEFINED;
}

void VehicleGatewayBetaflight::takeoff() {}

void VehicleGatewayBetaflight::land() {}

void VehicleGatewayBetaflight::go_to_waypoint() {}

void VehicleGatewayBetaflight::transition_to_fw() {}

void VehicleGatewayBetaflight::transition_to_mc() {}

void VehicleGatewayBetaflight::set_local_position_setpoint(
  float /*x*/, float /*y*/, float /*z*/, float /*yaw*/) {}

/// Documentation inherited
void VehicleGatewayBetaflight::set_local_velocity_setpoint(
  float /*vx*/, float /*vy*/, float /*vz*/, float /*yaw_rate*/) {}

void VehicleGatewayBetaflight::set_offboard_control_mode(
  vehicle_gateway::CONTROLLER_TYPE /*type*/) {}

void VehicleGatewayBetaflight::set_offboard_mode() {}

void VehicleGatewayBetaflight::set_ground_speed(float /*speed*/) {}

float VehicleGatewayBetaflight::get_ground_speed()
{
  return 0.0;
}

void VehicleGatewayBetaflight::go_to_latlon(double lat, double lon, float alt) {}
std::vector<double> VehicleGatewayBetaflight::get_latlon()
{
  return {-1, -1, -1};
}

}  // namespace vehicle_gateway_betaflight
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  vehicle_gateway_betaflight::VehicleGatewayBetaflight,
  vehicle_gateway::VehicleGateway)
