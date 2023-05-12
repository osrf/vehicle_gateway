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

#include <cmath>
#include <iostream>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_gateway_px4/vehicle_gateway_px4.hpp>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using namespace std;

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

class VehicleGatewayCpp
{
public:
  VehicleGatewayCpp()
  {
    this->loader_ = std::make_shared<pluginlib::ClassLoader<vehicle_gateway::VehicleGateway>>(
      "vehicle_gateway", "vehicle_gateway::VehicleGateway");
    this->gateway_ = this->loader_->createSharedInstance("vehicle_gateway_px4::VehicleGatewayPX4");
    this->gateway_->init(0, nullptr);
  }

  // todo(anyone) move this into the vehicle_gateway interface
  void arm_sync()
  {
    while (this->gateway_->get_arming_state() != vehicle_gateway::ARMING_STATE::ARMED &&
      rclcpp::ok())
    {
      this->gateway_->arm();
      usleep(1e5);  // 100 ms
    }
  }

  // todo(anyone) move this into the vehicle_gateway interface
  void disarm_sync()
  {
    while (this->gateway_->get_arming_state() != vehicle_gateway::ARMING_STATE::STANDBY &&
      rclcpp::ok())
    {
      this->gateway_->disarm();
      usleep(1e5);  // 100 ms
    }
  }

  void go_to_latlon_sync(
    double lat, double lon, double alt, double latlon_threshold = 0.5,
    double alt_threshold = 0.5)
  {
    this->gateway_->go_to_latlon(lat, lon, alt);
    while (true) {
      const auto current_latlon = this->gateway_->get_latlon();
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

  void offboard_mode_go_to_local_setpoint_sync(
    double x,
    double y,
    double alt,
    double yaw = numeric_limits<float>::quiet_NaN(),
    double airspeeed = 15.0,
    double distance_threshold = 10.0,
    vehicle_gateway::CONTROLLER_TYPE controller_type = vehicle_gateway::CONTROLLER_TYPE::POSITION)
  {
    while (true) {
      this->gateway_->set_offboard_control_mode(vehicle_gateway::CONTROLLER_TYPE::POSITION);
      this->gateway_->set_local_position_setpoint(x, y, alt, yaw);
      this->gateway_->set_airspeed(airspeeed);

      float current_ned_x, current_ned_y, current_ned_z;
      this->gateway_->get_local_position(current_ned_x, current_ned_y, current_ned_z);
      const auto dx = x - current_ned_x;
      const auto dy = y - current_ned_y;
      const auto dalt = alt - current_ned_z;
      const auto distance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dalt, 2));

      if (distance < distance_threshold) {
        break;
      }

      usleep(1e5); // 100 ms
    }
  }

  void transition_to_offboard_sync()
  {
    for (int i = 0; i < 20; i++) {
      this->gateway_->set_local_position_setpoint(
        0.0, 0.0,
        numeric_limits<float>::quiet_NaN(), numeric_limits<float>::quiet_NaN());
      this->gateway_->set_offboard_control_mode(vehicle_gateway::CONTROLLER_TYPE::POSITION);
      usleep(1e5);  // 100 ms
    }
    this->gateway_->set_offboard_mode();
  }

  void transition_to_multicopter_sync()
  {
    while (this->gateway_->get_vtol_state() != vehicle_gateway::VTOL_STATE::MC && rclcpp::ok()) {
      this->gateway_->transition_to_mc();
      usleep(1e5);  // 100 ms
    }
  }

  void transition_to_fixed_wing_sync()
  {
    while (this->gateway_->get_vtol_state() != vehicle_gateway::VTOL_STATE::FW && rclcpp::ok()) {
      this->gateway_->transition_to_fw();
      usleep(1e5);  // 100 ms
    }
  }

  void destroy()
  {
    if (this->gateway_) {
      this->gateway_->destroy();
      this->gateway_ = nullptr;
    }
    if (this->loader_) {
      this->loader_->unloadLibraryForClass(
        "vehicle_gateway_px4::VehicleGatewayPX4");
      this->loader_ = nullptr;
    }
  }

  std::shared_ptr<vehicle_gateway::VehicleGateway> gateway_;

private:
  std::shared_ptr<pluginlib::ClassLoader<vehicle_gateway::VehicleGateway>> loader_;
};


int main(int argc, const char * argv[])
{
  rclcpp::init(argc, argv);

  const auto vg = std::make_shared<VehicleGatewayCpp>();
  const float TARGET_ATTITUDE = 30.0f;

  cout << "Arming..." << endl;
  vg->arm_sync();
  sleep(2);

  const auto home_position = vg->gateway_->get_latlon();
  if (home_position.size() != 3) {
    throw std::runtime_error("home_position should have three elements: lat, lon, alt");
  }
  cout << "Home altitude: " << home_position[2] << endl;

  cout << "Takeoff!" << endl;
  vg->gateway_->takeoff();
  sleep(1);

  vg->go_to_latlon_sync(home_position[0], home_position[1], home_position[2] + TARGET_ATTITUDE);

  vg->transition_to_fixed_wing_sync();

  cout << "Begin transitioning to Offboard control..." << endl;
  vg->transition_to_offboard_sync();

  cout << "Enabled position controller" << endl;
  auto target_north = 200.0,
    target_east = 0.0;

  cout << "Flying to first waypoint..." << endl;
  vg->offboard_mode_go_to_local_setpoint_sync(
    target_north, target_east, -TARGET_ATTITUDE,
    numeric_limits<float>::quiet_NaN(), 15);
  cout << "Flying to second waypoint..." << endl;
  vg->offboard_mode_go_to_local_setpoint_sync(
    -target_north, target_east, -TARGET_ATTITUDE,
    numeric_limits<float>::quiet_NaN(), 20);
  cout << "Flying home..." << endl;
  vg->offboard_mode_go_to_local_setpoint_sync(
    0, 0, -TARGET_ATTITUDE,
    numeric_limits<float>::quiet_NaN(), 20);

  cout << "Switching back to hold mode..." << endl;
  vg->gateway_->set_onboard_mode();

  cout << "Transitioning to multicopter..." << endl;
  vg->transition_to_multicopter_sync();
  cout << "VTOL state: " << vg->gateway_->get_vtol_state() << endl;
  sleep(1);

  vg->go_to_latlon_sync(home_position[0], home_position[1], home_position[2] + TARGET_ATTITUDE);

  cout << "Landing..." << endl;
  vg->gateway_->land();

  cout << "Disarming..." << endl;
  vg->disarm_sync();

  vg->destroy();
  cout << "Demo complete." << endl;

  rclcpp::shutdown();
  return 0;
}
