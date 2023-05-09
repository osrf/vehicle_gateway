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

#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_loader.hpp>

#include <vehicle_gateway_px4/vehicle_gateway_px4.hpp>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using namespace std;

const double EARTH_RADIUS = 6378100 ; // meters

double calc_distance_latlon(double lat_1, double lon_1, double lat_2, double lon_2) {
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

  void arm_sync() {
    while (this->gateway_->get_arming_state() != vehicle_gateway::ARMING_STATE::ARMED) {
      this->gateway_->arm();
      usleep(1e5);  // 100 ms
    }
  }

  void disarm_sync() {
    while (this->gateway_->get_arming_state() != vehicle_gateway::ARMING_STATE::STANDBY) {
      this->gateway_->disarm();
      usleep(1e5);  // 100 ms
    }
  }

  void transition_to_multicopter_sync() {
    while (this->gateway_->get_vtol_state() != vehicle_gateway::VTOL_STATE::MC) {
      this->gateway_->transition_to_mc();
      usleep(1e5);  // 100 ms
    }
  }

  void transition_to_fixed_wing_sync() {
    while (this->gateway_->get_vtol_state() != vehicle_gateway::VTOL_STATE::FW) {
      this->gateway_->transition_to_fw();
      usleep(1e5);  // 100 ms
    }
  }

  void destroy(){
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

  auto dist = calc_distance_latlon(12.5, 12.5, 12.6, 12.6);
  cout << "Distance: " << dist << endl;

  const auto vg = std::make_shared<VehicleGatewayCpp>();
  const auto TARGET_ATTITUDE = 30.0;

  cout << "Arming..." << endl;
  vg->arm_sync();
  sleep(2);

  const auto home_position = vg->gateway_->get_latlon();
  if (home_position.size() != 3) {
    throw std::runtime_error("home_position should have three elements: lat, lon, alt");
  }
  cout << "Home altitude: " << home_position[2] << endl;

  cout <<"Takeoff!" << endl;
  vg->gateway_->takeoff();


  sleep(10);


  cout <<"Landing..." << endl;
  vg->gateway_->land();

  cout << "Disarming..." << endl;
  vg->disarm_sync();

  vg->destroy();
  cout << "Demo complete." << endl;

  rclcpp::shutdown();
  return 0;
}
