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
      auto set_arm_disarm_reason = [](uint8_t reason)
      {
        vehicle_gateway::ARM_DISARM_REASON value;
        switch (reason) {
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_TRANSITION_TO_STANDBY:
            value = vehicle_gateway::ARM_DISARM_REASON::TRANSITION_TO_STANDBY;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_RC_STICK:
            value = vehicle_gateway::ARM_DISARM_REASON::RC_STICK;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_RC_SWITCH:
            value = vehicle_gateway::ARM_DISARM_REASON::RC_SWITCH;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_COMMAND_INTERNAL:
            value = vehicle_gateway::ARM_DISARM_REASON::COMMAND_INTERNAL;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_COMMAND_EXTERNAL:
            value = vehicle_gateway::ARM_DISARM_REASON::COMMAND_EXTERNAL;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_MISSION_START:
            value = vehicle_gateway::ARM_DISARM_REASON::MISSION_START;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_SAFETY_BUTTON:
            value = vehicle_gateway::ARM_DISARM_REASON::SAFETY_BUTTON;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_AUTO_DISARM_LAND:
            value = vehicle_gateway::ARM_DISARM_REASON::AUTO_DISARM_LAND;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT:
            value = vehicle_gateway::ARM_DISARM_REASON::AUTO_DISARM_PREFLIGHT;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_KILL_SWITCH:
            value = vehicle_gateway::ARM_DISARM_REASON::KILL_SWITCH;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_LOCKDOWN:
            value = vehicle_gateway::ARM_DISARM_REASON::LOCKDOWN;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_FAILURE_DETECTOR:
            value = vehicle_gateway::ARM_DISARM_REASON::FAILURE_DETECTOR;
            break;
          case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_SHUTDOWN:
            value = vehicle_gateway::ARM_DISARM_REASON::SHUTDOWN;
            break;
        }
        return value;
      };

      this->arm_reason_ = set_arm_disarm_reason(msg->latest_arming_reason);
      this->disarm_reason_ = set_arm_disarm_reason(msg->latest_disarming_reason);

      switch (msg->failure_detector_status) {
        case px4_msgs::msg::VehicleStatus::FAILURE_NONE:
          this->failure_ = vehicle_gateway::FAILURE::NONE;
          break;
        case px4_msgs::msg::VehicleStatus::FAILURE_ROLL:
          this->failure_ = vehicle_gateway::FAILURE::ROLL;
          break;
        case px4_msgs::msg::VehicleStatus::FAILURE_PITCH:
          this->failure_ = vehicle_gateway::FAILURE::PITCH;
          break;
        case px4_msgs::msg::VehicleStatus::FAILURE_ALT:
          this->failure_ = vehicle_gateway::FAILURE::ALT;
          break;
        case px4_msgs::msg::VehicleStatus::FAILURE_ARM_ESC:
          this->failure_ = vehicle_gateway::FAILURE::ARM_ESC;
          break;
        case px4_msgs::msg::VehicleStatus::FAILURE_BATTERY:
          this->failure_ = vehicle_gateway::FAILURE::BATTERY;
          break;
        case px4_msgs::msg::VehicleStatus::FAILURE_IMBALANCED_PROP:
          this->failure_ = vehicle_gateway::FAILURE::IMBALANCED_PROP;
          break;
        case px4_msgs::msg::VehicleStatus::FAILURE_MOTOR:
          this->failure_ = vehicle_gateway::FAILURE::MOTOR;
          break;
        case px4_msgs::msg::VehicleStatus::FAILURE_EXT:
          this->failure_ = vehicle_gateway::FAILURE::EXT;
          break;
        default:
          this->failure_ = vehicle_gateway::FAILURE::NONE;
      }

      switch (msg->vehicle_type) {
        case px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_UNKNOWN:
          this->vehicle_type_ = vehicle_gateway::VEHICLE_TYPE::UNKNOWN;
          break;
        case px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING:
          this->vehicle_type_ = vehicle_gateway::VEHICLE_TYPE::ROTARY_WING;
          break;
        case px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING:
          this->vehicle_type_ = vehicle_gateway::VEHICLE_TYPE::FIXED_WING;
          break;
        case px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROVER:
          this->vehicle_type_ = vehicle_gateway::VEHICLE_TYPE::ROVER;
          break;
        case px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_AIRSHIP:
          this->vehicle_type_ = vehicle_gateway::VEHICLE_TYPE::AIRSHIP;
          break;
        default:
          this->vehicle_type_ = vehicle_gateway::VEHICLE_TYPE::UNKNOWN;
      }

      switch (msg->arming_state) {
        case px4_msgs::msg::VehicleStatus::ARMING_STATE_INIT:
          this->arming_state_ = vehicle_gateway::ARMING_STATE::INIT;
          break;
        case px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY:
          this->arming_state_ = vehicle_gateway::ARMING_STATE::STANDBY;
          break;
        case px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED:
          this->arming_state_ = vehicle_gateway::ARMING_STATE::ARMED;
          break;
        case px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY_ERROR:
          this->arming_state_ = vehicle_gateway::ARMING_STATE::STANDBY_ERROR;
          break;
        case px4_msgs::msg::VehicleStatus::ARMING_STATE_SHUTDOWN:
          this->arming_state_ = vehicle_gateway::ARMING_STATE::SHUTTEDDOWN;
          break;
        case px4_msgs::msg::VehicleStatus::ARMING_STATE_IN_AIR_RESTORE:
          this->arming_state_ = vehicle_gateway::ARMING_STATE::IN_AIR_RESTORE;
          break;
        case px4_msgs::msg::VehicleStatus::ARMING_STATE_MAX:
          this->arming_state_ = vehicle_gateway::ARMING_STATE::MAX;
          break;
        default:
          this->arming_state_ = vehicle_gateway::ARMING_STATE::MAX;
      }

      switch (msg->nav_state) {
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::MANUAL;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::ALTCTL;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::POSCTL;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::AUTO_MISSION;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::AUTO_LOITER;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::AUTO_RTL;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ACRO:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::ACRO;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_TERMINATION:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::TERMINATION;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::OFFBOARD;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_STAB:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::STAB;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::AUTO_TAKEOFF;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::AUTO_LAND;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::AUTO_FOLLOW_TARGET;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::AUTO_PRECLAND;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::ORBIT;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::AUTO_VTOL_TAKEOFF;
          break;
        case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::DESCEND;
          break;
        default:
          this->flight_mode_ = vehicle_gateway::FLIGHT_MODE::UNKNOWN_MODE;
      }
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

  this->vehicle_odometry_sub_ =
    this->px4_node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry",
    qos_profile,
    [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
      this->odom_timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
        std::chrono::nanoseconds(msg->timestamp));
      this->ground_speed_ = std::sqrt(
        std::pow(msg->velocity[0], 2) + std::pow(
          msg->velocity[1],
          2));

      current_pos_x_ = msg->position[0];
      current_pos_y_ = msg->position[1];
      current_pos_z_ = msg->position[2];
      current_vel_x_ = msg->velocity[0];
      current_vel_y_ = msg->velocity[1];
      current_vel_z_ = msg->velocity[2];
    });

  this->vehicle_command_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", qos_profile);
  this->vehicle_trajectory_setpoint_pub_ =
    this->px4_node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", qos_profile);
  this->vehicle_offboard_control_mode_pub_ =
    this->px4_node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", qos_profile);
}

VehicleGatewayPX4::~VehicleGatewayPX4()
{
  this->destroy();
}

vehicle_gateway::VEHICLE_TYPE VehicleGatewayPX4::get_vehicle_type()
{
  return this->vehicle_type_;
}

vehicle_gateway::FAILURE VehicleGatewayPX4::get_failure()
{
  return this->failure_;
}

void VehicleGatewayPX4::destroy()
{
  if (this->exec_) {
    this->exec_->cancel();
    this->exec_ = nullptr;
    rclcpp::shutdown();
    this->spin_thread_.join();
  }
}

vehicle_gateway::ARM_DISARM_REASON VehicleGatewayPX4::get_disarm_reason()
{
  return this->disarm_reason_;
}

vehicle_gateway::ARM_DISARM_REASON VehicleGatewayPX4::get_arm_reason()
{
  return this->arm_reason_;
}

vehicle_gateway::FLIGHT_MODE VehicleGatewayPX4::get_flight_mode()
{
  return this->flight_mode_;
}

vehicle_gateway::ARMING_STATE VehicleGatewayPX4::get_arming_state()
{
  return this->arming_state_;
}

void VehicleGatewayPX4::send_command(
  uint32_t command, uint8_t target_system, uint8_t target_component, uint8_t source_system,
  uint8_t source_component, uint8_t confirmation, bool from_external,
  float param1, float param2, float param3,
  float param4, float param5, float param6,
  float param7)
{
  px4_msgs::msg::VehicleCommand msg_vehicle_command;
  msg_vehicle_command.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000.0;
  msg_vehicle_command.command = command;

  msg_vehicle_command.param1 = param1;
  msg_vehicle_command.param2 = param2;
  msg_vehicle_command.param3 = param3;
  msg_vehicle_command.param4 = param4;
  msg_vehicle_command.param5 = param5;
  msg_vehicle_command.param6 = param6;
  msg_vehicle_command.param7 = param7;
  msg_vehicle_command.confirmation = confirmation;
  msg_vehicle_command.source_system = source_system;
  msg_vehicle_command.target_system = target_system;
  msg_vehicle_command.target_component = target_component;
  msg_vehicle_command.from_external = from_external;
  msg_vehicle_command.source_component = source_component;

  this->vehicle_command_pub_->publish(msg_vehicle_command);
}

void VehicleGatewayPX4::arm()
{
  this->send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    this->target_system_,
    1,
    255,
    0,
    1,
    true,
    1.0f);
}

void VehicleGatewayPX4::disarm()
{
  this->send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    this->target_system_,
    1,
    255,
    0,
    1,
    true,
    0.0f);
}

void VehicleGatewayPX4::set_offboard_mode()
{
  this->send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
    this->target_system_,
    1,
    255,
    1,
    1,
    true,
    1.0f,
    6.0f);
}

float VehicleGatewayPX4::get_ground_speed()
{
  return this->ground_speed_;
}

void VehicleGatewayPX4::takeoff()
{
  this->send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
    this->target_system_,
    1,
    255,
    1,
    1,
    true,
    0.1f,
    0,
    0,
    1.57,  // orientation
    this->lat_ * 1e-7,
    this->lon_ * 1e-7,
    this->alt_ + 5.0f);
}

void VehicleGatewayPX4::land()
{
  this->send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,
    this->target_system_,
    1,
    255,
    1,
    1,
    true,
    0.1f,
    0,
    0,
    1.57,  // orientation
    this->lat_ * 1e-7,
    this->lon_ * 1e-7);
}

void VehicleGatewayPX4::transition_to_fw()
{
  this->send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION,
    this->target_system_,
    1,
    255,
    1,
    1,
    true,
    4.0f,
    1.0f);
}

void VehicleGatewayPX4::transition_to_mc()
{
  this->send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION,
    this->target_system_,
    1,
    255,
    1,
    1,
    true,
    3.0f,
    1.0f);
}

void VehicleGatewayPX4::set_local_velocity_setpoint(float vx, float vy, float vz, float yaw_rate)
{
  px4_msgs::msg::TrajectorySetpoint msg;

  msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

  msg.position[0] = std::numeric_limits<float>::quiet_NaN();
  msg.position[1] = std::numeric_limits<float>::quiet_NaN();
  msg.position[2] = std::numeric_limits<float>::quiet_NaN();
  msg.yaw = std::numeric_limits<float>::quiet_NaN();

  msg.velocity[0] = vx;
  msg.velocity[1] = vy;
  msg.velocity[2] = vz;
  msg.yawspeed = yaw_rate;
  this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void VehicleGatewayPX4::set_local_position_setpoint(float x, float y, float z, float yaw)
{
  px4_msgs::msg::TrajectorySetpoint msg;

  msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

  msg.position[0] = x;
  msg.position[1] = y;
  msg.position[2] = z;
  msg.yaw = yaw;
  this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void VehicleGatewayPX4::set_offboard_control_mode(vehicle_gateway::CONTROLLER_TYPE type)
{
  px4_msgs::msg::OffboardControlMode msg;
  msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

  if (type == vehicle_gateway::CONTROLLER_TYPE::POSITION) {
    msg.position = true;
    msg.velocity = false;
  } else if (type == vehicle_gateway::CONTROLLER_TYPE::VELOCITY) {
    msg.position = false;
    msg.velocity = true;
  } else {
    msg.position = false;
    msg.velocity = false;
    RCLCPP_INFO(this->px4_node_->get_logger(), "No controller is defined");
  }
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;

  this->vehicle_offboard_control_mode_pub_->publish(msg);
}

void VehicleGatewayPX4::go_to_waypoint()
{
}

}  // namespace vehicle_gateway_px4
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vehicle_gateway_px4::VehicleGatewayPX4, vehicle_gateway::VehicleGateway)
