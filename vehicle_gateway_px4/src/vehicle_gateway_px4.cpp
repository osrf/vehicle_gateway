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

#include <zenoh.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "px4_msgs/msg/vehicle_local_position_setpoint.hpp"
#include "tf2/utils.h"
#include "nlohmann/json.hpp"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using std::string;
using nlohmann::json;

namespace vehicle_gateway_px4
{

const double EARTH_RADIUS = 6378100;  // meters

double calc_distance_latlon(double lat_1, double lon_1, double lat_2, double lon_2)
{
  // This uses a planar approximation, not the real trigonometry. This
  // approximation is just fine for short distances, which is all we're
  // currently considering.

  double dx = (EARTH_RADIUS * M_PI / 180) * (lon_2 - lon_1) * cos(lat_1);
  double dy = (EARTH_RADIUS * M_PI / 180) * (lat_2 - lat_1);

  return sqrt(dx * dx + dy * dy);
}

VehicleGatewayPX4::VehicleGatewayPX4()
: VehicleGateway()
{
  zenoh_session_ = z_session_null();
  zenoh_state_pub_ = z_publisher_null();
}

void VehicleGatewayPX4::set_vehicle_id(unsigned int _vehicle_id)
{
  this->vehicle_id_ = _vehicle_id;
}

unsigned int VehicleGatewayPX4::get_vehicle_id()
{
  return this->vehicle_id_;
}

void VehicleGatewayPX4::init(int argc, const char ** argv)
{
  if (argc != 0 && argv != nullptr) {
    rclcpp::init(argc, argv);
  }
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

  std::string vehicle_id_prefix{};

  if (this->vehicle_id_ != 0) {
    vehicle_id_prefix = "/px4_" + std::to_string(this->vehicle_id_);
    this->target_system_ = this->vehicle_id_ + 1;
  }

  this->vehicle_status_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::VehicleStatus>(
    vehicle_id_prefix + "/fmu/out/vehicle_status",
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
          default:
            value = vehicle_gateway::ARM_DISARM_REASON::ARM_DISARM_REASON_NONE;
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

      // Use several fields in vehicle_status to determine vtol state since
      // it seems that /fmu/out/vtol_vehicle_status is no longer being sent (?)
      if (msg->in_transition_mode) {
        if (msg->in_transition_to_fw) {
          this->vtol_state_ = vehicle_gateway::VTOL_STATE::TRANSITION_TO_FW;
        } else {
          this->vtol_state_ = vehicle_gateway::VTOL_STATE::TRANSITION_TO_MC;
        }
      } else {
        switch (msg->vehicle_type) {
          case px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING:
            this->vtol_state_ = vehicle_gateway::VTOL_STATE::MC;
            break;
          case px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING:
            this->vtol_state_ = vehicle_gateway::VTOL_STATE::FW;
            break;
          default:
            this->vtol_state_ = vehicle_gateway::VTOL_STATE::UNDEFINED;
        }
      }
    });

  this->vehicle_sensor_gps_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::SensorGps>(
    vehicle_id_prefix + "/fmu/out/vehicle_gps_position",
    qos_profile,
    [this](px4_msgs::msg::SensorGps::ConstSharedPtr msg) {
      // 1e-7 and 1e-3 comes from:
      // https://github.com/PX4/px4_msgs/blob/4db0a3f14ea81b9de7511d738f8ad9bd8ae5b3ad/msg/SensorGps.msg#L8-L10
      this->lat_ = msg->lat * 1e-7;
      this->lon_ = msg->lon * 1e-7;
      this->alt_ = msg->alt * 1e-3;
    });

  this->vehicle_timesync_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::TimesyncStatus>(
    vehicle_id_prefix + "/fmu/out/timesync_status",
    qos_profile,
    [this](px4_msgs::msg::TimesyncStatus::ConstSharedPtr msg) {
      this->timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
        std::chrono::nanoseconds(msg->timestamp));
    });

  this->vehicle_odometry_sub_ =
    this->px4_node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
    vehicle_id_prefix + "/fmu/out/vehicle_odometry",
    qos_profile,
    [this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
      this->odom_timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
        std::chrono::nanoseconds(msg->timestamp));
      this->ground_speed_ = std::sqrt(
        std::pow(msg->velocity[0], 2) + std::pow(
          msg->velocity[1],
          2));

      this->current_pos_x_ = msg->position[0];
      this->current_pos_y_ = msg->position[1];
      this->current_pos_z_ = msg->position[2];
      this->current_vel_x_ = msg->velocity[0];
      this->current_vel_y_ = msg->velocity[1];
      this->current_vel_z_ = msg->velocity[2];

      // if the quaternion is valid, extract the euler angles for convenience
      if (msg->q[0] != NAN) {
        double y = 0, p = 0, r = 0;
        // the ordering is different: PX4 does wxyz, TF2/Bullet does xyzw
        tf2::getEulerYPR(
          tf2::Quaternion(msg->q[1], msg->q[2], msg->q[3], msg->q[0]),
          y, p, r);
        this->yaw_ = static_cast<float>(y);
        this->pitch_ = static_cast<float>(p);
        this->roll_ = static_cast<float>(r);
      }
    });

  this->airspeed_sub_ =
    this->px4_node_->create_subscription<px4_msgs::msg::Airspeed>(
    vehicle_id_prefix + "/fmu/out/airspeed",
    qos_profile,
    [this](px4_msgs::msg::Airspeed::ConstSharedPtr msg) {
      this->airspeed_ = msg->true_airspeed_m_s;
    });

  this->vehicle_rates_setpoint_pub_ =
    this->px4_node_->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
    vehicle_id_prefix + "/fmu/in/vehicle_rates_setpoint", qos_profile);
  this->vehicle_command_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::VehicleCommand>(
    vehicle_id_prefix + "/fmu/in/vehicle_command", qos_profile);
  this->vehicle_trajectory_setpoint_pub_ =
    this->px4_node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    vehicle_id_prefix + "/fmu/in/trajectory_setpoint", qos_profile);
  this->vehicle_offboard_control_mode_pub_ =
    this->px4_node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
    vehicle_id_prefix + "/fmu/in/offboard_control_mode", qos_profile);
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
    // this is the type of code you write just before the velociraptors attack
    for (int i = 0; i < 42; i++) {
      this->exec_->cancel();
      usleep(100);  // sleep a bit to force a thread context switch
    }
    // we've asked it to shut down many times, so clearly now we're good lol
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

vehicle_gateway::VTOL_STATE VehicleGatewayPX4::get_vtol_state()
{
  return this->vtol_state_;
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
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    1.0f);
}

void VehicleGatewayPX4::arm_sync()
{
  while (get_arming_state() != vehicle_gateway::ARMING_STATE::ARMED &&
    rclcpp::ok())
  {
    this->arm();
    usleep(1e5);  // 100 ms
  }
}

void VehicleGatewayPX4::disarm()
{
  this->send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    0.0f);
}

void VehicleGatewayPX4::disarm_sync()
{
  while (get_arming_state() != vehicle_gateway::ARMING_STATE::STANDBY &&
    rclcpp::ok())
  {
    this->disarm();
    usleep(1e5);  // 100 ms
  }
}

void VehicleGatewayPX4::set_offboard_mode()
{
  this->send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    1.0f,
    6.0f);
}

void VehicleGatewayPX4::set_onboard_mode()
{
  // Sometimes this is called "mission" or "main" mode in PX4 code/docs
  // essentially this means the PX4 microcontroller itself is flying the
  // vehicle and does not need a stream of commands from another computer
  this->send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    1.0f,
    4.0f,   // main mode: "auto"
    3.0f);  // sub-mode: 1="auto ready" ? 3="auto loiter", 4="auto mission"
}

float VehicleGatewayPX4::get_ground_speed()
{
  return this->ground_speed_;
}

float VehicleGatewayPX4::get_airspeed()
{
  return this->airspeed_;
}

void VehicleGatewayPX4::takeoff()
{
  this->send_command(
    // https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    0.1f,  // Minimum pitch (if airspeed sensor present), desired pitch without sensor (degrees)
    0,     // Empty
    0,     // Empty
    1.57,  // Yaw angle (degrees)
    this->lat_,  // Latitude
    this->lon_,  // Longitude
    this->alt_ + 5.0f);  // Altitude (meters)
}

void VehicleGatewayPX4::land()
{
  this->send_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    0.1f,
    0,
    0,
    1.57,  // orientation
    this->lat_,
    this->lon_);
}

void VehicleGatewayPX4::transition_to_fw()
{
  this->send_command(
    // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_VTOL_TRANSITION
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    vehicle_gateway::VTOL_STATE::FW,  // The target VTOL state
    1.0f);  // Force immediate transition to the specified MAV_VTOL_STATE
}

void VehicleGatewayPX4::transition_to_mc_sync()
{
  while (this->get_vtol_state() != vehicle_gateway::VTOL_STATE::MC && rclcpp::ok()) {
    this->transition_to_mc();
    usleep(1e5);  // 100 ms
  }
}

void VehicleGatewayPX4::transition_to_mc()
{
  this->send_command(
    // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_VTOL_TRANSITION
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    vehicle_gateway::VTOL_STATE::MC,  // The target VTOL state
    0.0f);  // don't force immediate transition; allow PX4 to slow down first
}

void VehicleGatewayPX4::transition_to_fw_sync()
{
  while (get_vtol_state() != vehicle_gateway::VTOL_STATE::FW && rclcpp::ok()) {
    transition_to_fw();
    usleep(1e5);  // 100 ms
  }
}

void VehicleGatewayPX4::go_to_latlon(double lat, double lon, float alt_amsl)
{
  this->send_command(
    // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPOSITION
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    -1.0f,  // ground speed (m/s), less than 0 (-1) for default
    NAN,    // bitmask TODO(anyone) MAV_DO_REPOSITION_FLAGS_CHANGE_MODE?
    0.0f,   // loiter radius for plans (meters)
    NAN,    // yaw heading (deg), NaN to use current system yaw heading.
            // For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
    lat,    // latitude
    lon,    // longitude
    alt_amsl  // altitude above mean sea level (AMSL) (m)
  );
}

void VehicleGatewayPX4::go_to_latlon_sync(
  double lat, double lon, double alt, double latlon_threshold,
  double alt_threshold)
{
  go_to_latlon(lat, lon, alt);
  while (rclcpp::ok()) {
    const auto current_latlon = get_latlon();
    if (current_latlon.size() != 3) {
      throw std::runtime_error("current_latlon should have three elements: lat, lon, alt");
    }
    const auto current_lat = current_latlon[0];
    const auto current_lon = current_latlon[1];
    const auto current_alt = current_latlon[2];
    const auto latlon_distance = calc_distance_latlon(lat, lon, current_lat, current_lon);
    const auto alt_distance = sqrt(pow(alt - current_alt, 2));
    if (latlon_distance < latlon_threshold && alt_distance < alt_threshold) {
      break;
    }
    usleep(5e5);  // 500 ms
  }
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

  msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
  msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
  msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();

  this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void VehicleGatewayPX4::set_local_position_setpoint(float x, float y, float z, float yaw)
{
  px4_msgs::msg::TrajectorySetpoint msg;

  msg.position[0] = x;
  msg.position[1] = y;
  msg.position[2] = z;
  msg.yaw = yaw;

  // non-NaN velocity and acceleration fields are used as feedforward terms.
  // We will just set them all to NaN, to keep this API simple.

  msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
  msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
  msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
  msg.yawspeed = std::numeric_limits<float>::quiet_NaN();

  msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
  msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
  msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();

  this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void VehicleGatewayPX4::offboard_mode_go_to_local_setpoint_sync(
  double x,
  double y,
  double alt,
  double yaw,
  double airspeeed,
  double distance_threshold,
  vehicle_gateway::CONTROLLER_TYPE controller_type)
{
  while (rclcpp::ok()) {
    set_offboard_control_mode(controller_type);
    set_local_position_setpoint(x, y, alt, yaw);
    set_airspeed(airspeeed);

    float current_ned_x, current_ned_y, current_ned_z;
    get_local_position(current_ned_x, current_ned_y, current_ned_z);
    const auto dx = x - current_ned_x;
    const auto dy = y - current_ned_y;
    const auto dalt = alt - current_ned_z;
    const auto distance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dalt, 2));

    if (distance < distance_threshold) {
      break;
    }

    usleep(1e5);  // 100 ms
  }
}

void VehicleGatewayPX4::set_ground_speed(float speed)
{
  this->set_speed(speed, true);
}

void VehicleGatewayPX4::set_airspeed(float speed)
{
  this->set_speed(speed, false);
}

void VehicleGatewayPX4::set_speed(float speed, bool is_ground_speed)
{
  float speed_type = is_ground_speed ? 1.0f : 0.0f;  // true = ground speed, false = air speed
  this->send_command(
    // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    speed_type,  // Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
    speed,       // Speed (-1 indicates no change, -2 indicates return to default vehicle speed)
    -1.0f);      // Throttle (-1 indicates no change, -2 indicates return to default throttle value)
}

void VehicleGatewayPX4::transition_to_offboard_sync()
{
  for (int i = 0; i < 20; i++) {
    set_local_position_setpoint(
      0.0,
      0.0,
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN());
    set_offboard_control_mode(vehicle_gateway::CONTROLLER_TYPE::POSITION);
    usleep(1e5);  // 100 ms
  }
  set_offboard_mode();
}

void VehicleGatewayPX4::set_offboard_control_mode(vehicle_gateway::CONTROLLER_TYPE type)
{
  px4_msgs::msg::OffboardControlMode msg;
  msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.actuator = false;

  if (type == vehicle_gateway::CONTROLLER_TYPE::POSITION) {
    msg.position = true;
  } else if (type == vehicle_gateway::CONTROLLER_TYPE::VELOCITY) {
    msg.velocity = true;
  } else if (type == vehicle_gateway::CONTROLLER_TYPE::BODY_RATES) {
    msg.body_rate = true;
  } else {
    RCLCPP_WARN(this->px4_node_->get_logger(), "No controller is defined");
  }

  this->vehicle_offboard_control_mode_pub_->publish(msg);
}

bool VehicleGatewayPX4::set_body_rates_and_thrust_setpoint(
  float roll_rate, float pitch_rate, float yaw_rate, float thrust)
{
  px4_msgs::msg::VehicleRatesSetpoint msg;

  msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;
  msg.roll = roll_rate;
  msg.pitch = pitch_rate;
  msg.yaw = yaw_rate;

  if (this->vehicle_type_ == vehicle_gateway::VEHICLE_TYPE::FIXED_WING) {
    msg.thrust_body[0] = thrust;  // positive body X = forward
    msg.thrust_body[1] = 0;
    msg.thrust_body[2] = 0;
  } else {
    // it is either a multicopter or it's transitioning to/from multicopter
    msg.thrust_body[0] = 0;
    msg.thrust_body[1] = 0;
    msg.thrust_body[2] = thrust;  // negative = "up" (away from ground)
  }
  msg.reset_integral = false;

  this->vehicle_rates_setpoint_pub_->publish(msg);
  return true;
}

bool VehicleGatewayPX4::set_motors(std::vector<uint16_t>/*motor_values*/)
{
  return true;
}

void VehicleGatewayPX4::get_local_position(float & x, float & y, float & z)
{
  x = this->current_pos_x_;
  y = this->current_pos_y_;
  z = this->current_pos_z_;
}

void VehicleGatewayPX4::get_euler_rpy(float & roll, float & pitch, float & yaw)
{
  roll = this->roll_;
  pitch = this->pitch_;
  yaw = this->yaw_;
}

std::vector<double> VehicleGatewayPX4::get_latlon()
{
  return {this->lat_, this->lon_, this->alt_};
}

/// Documentation inherited
float VehicleGatewayPX4::get_altitude()
{
  return this->current_pos_z_;
}

bool VehicleGatewayPX4::create_multirobot_session(const char * config_filename)
{
  double tx_interval = 0.5;
  string zenoh_config;
  try {
    std::ifstream f(config_filename);
    const json j = json::parse(f);
    const json j_zenoh = j["zenoh"];
    zenoh_config = j_zenoh.dump();
    tx_interval = j["state_transmit_interval_seconds"].get<double>();
  } catch (...) {
    RCLCPP_FATAL(this->px4_node_->get_logger(), "unable to parse %s\n", config_filename);
    return false;
  }

  z_owned_config_t config = zc_config_from_str(zenoh_config.c_str());
  if (!z_check(config)) {
    RCLCPP_FATAL(
      this->px4_node_->get_logger(),
      "Unable to create zenoh config from %s",
      zenoh_config.c_str());
  }
  RCLCPP_INFO(
    this->px4_node_->get_logger(),
    "Created zenoh config successfully");
  zenoh_session_ = z_open(z_move(config));
  if (!z_check(zenoh_session_)) {
    RCLCPP_FATAL(
      this->px4_node_->get_logger(),
      "Unable to create zenoh session");
    return false;
  }
  RCLCPP_INFO(
    this->px4_node_->get_logger(),
    "Zenoh session opened successfully");
  RCLCPP_INFO(
    this->px4_node_->get_logger(),
    "Creating topic subscriptions for vehicle %d", get_vehicle_id());

  string state_key = string("vehicle_gateway/") + std::to_string(get_vehicle_id()) +
    string("/state");
  RCLCPP_INFO(
    this->px4_node_->get_logger(),
    "Declaring state publisher on Zenoh keyexpr %s", state_key.c_str());
  zenoh_state_pub_ = z_declare_publisher(
    z_loan(zenoh_session_), z_keyexpr(state_key.c_str()), NULL);
  if (!z_check(zenoh_state_pub_)) {
    RCLCPP_FATAL(
      this->px4_node_->get_logger(),
      "Unable to create zenoh state publisher");
    return false;
  }

  zenoh_transmit_timer_ = rclcpp::create_timer(
    this->px4_node_,
    this->px4_node_->get_clock(),
    rclcpp::Duration::from_seconds(tx_interval),
    [this]() {
      this->zenoh_transmit();
    });

  return true;
}

void VehicleGatewayPX4::zenoh_transmit()
{
  // todo: probably should wrap a mutex around this data block since
  // we have a multithreaded executor and it will get updated in parallel

  json j;
  j["id"] = this->vehicle_id_;
  j["east"] = this->current_pos_x_;
  j["north"] = this->current_pos_y_;
  j["down"] = this->current_pos_z_;
  j["airspeed"] = this->airspeed_;
  j["heading"] = this->yaw_;
  // TODO(ahcorde): add a timestamp that is relevant to multi-vehicle. Maybe GPS time
  // plus out local clock elapsed time since GPS time was last updated?
  string s = j.dump();

  z_publisher_put_options_t options = z_publisher_put_options_default();
  options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_JSON, NULL);
  z_publisher_put(
    z_loan(this->zenoh_state_pub_),
    reinterpret_cast<const uint8_t *>(s.c_str()),
    s.size() + 1,
    &options);
}

bool VehicleGatewayPX4::destroy_multirobot_session()
{
  if (!z_check(zenoh_session_)) {
    RCLCPP_WARN(
      this->px4_node_->get_logger(),
      "Zenoh session is not open; can't destroy it!");
    return false;
  }
  z_close(z_move(zenoh_session_));
  return true;
}

void * VehicleGatewayPX4::get_multirobot_session()
{
  return &zenoh_session_;
}

}  // namespace vehicle_gateway_px4
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vehicle_gateway_px4::VehicleGatewayPX4, vehicle_gateway::VehicleGateway)
