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

#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_gateway_px4/vehicle_gateway_px4.hpp>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using nlohmann::json;
using std::string;

class Follower
{
public:
  Follower(const int vehicle_id)
  {
    this->loader_ = std::make_shared<pluginlib::ClassLoader<vehicle_gateway::VehicleGateway>>(
      "vehicle_gateway", "vehicle_gateway::VehicleGateway");
    this->gateway_ = this->loader_->createSharedInstance("vehicle_gateway_px4::VehicleGatewayPX4");
    this->gateway_->set_vehicle_id(vehicle_id);
    this->gateway_->init(0, nullptr);
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
  void state_handler(const z_sample_t * sample);

private:
  std::shared_ptr<pluginlib::ClassLoader<vehicle_gateway::VehicleGateway>> loader_;
};

void Follower::state_handler(const z_sample_t * sample)
{
  json j;
  try {
    j = json::parse(
      std::string(
        reinterpret_cast<const char *>(sample->payload.start),
        sample->payload.len));
  } catch (...) {
    printf("error parsing json\n");
    return;
  }
  // todo: test for required fields
  const int sender_id = j["id"].get<int>();
  if (sender_id != 1) {
    printf("ignoring non-leader state (id %d)\n", sender_id);
    return;
  }
  const double leader_north = j["north"].get<double>();
  const double leader_east = j["east"].get<double>();
  const double leader_down = j["down"].get<double>();
  printf(
    "leader position: (%.3f, %.3f, %.3f)\n",
    leader_north, leader_east, leader_down);

  // TODO: here we shall do something smart to chase after the leader
}

void state_handler(const z_sample_t * sample, void * context)
{
  Follower * follower = reinterpret_cast<Follower *>(context);
  follower->state_handler(sample);
}

int main(int argc, const char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 3) {
    printf("usage: follower VEHICLE_ID CONFIG.json\n");
    return 1;
  }
  const int vehicle_id = atoi(argv[1]);
  const char * const config_filename = argv[2];

  const auto vg = std::make_shared<Follower>(vehicle_id);
  if (!vg->gateway_->create_multirobot_session(config_filename)) {
    printf("failed to create multirobot session!\n");
    return 1;
  }

  // note that ROS_DOMAIN_ID must be set equal to the domain that the
  // desired vehicle_id is living in. If not, this will silently be sad.
  // todo: detect this situation and print a helpful warning message.

  z_owned_closure_sample_t callback = z_closure(state_handler, NULL, vg.get());
  z_owned_session_t * zenoh_session = reinterpret_cast<z_owned_session_t *>(
    vg->gateway_->get_multirobot_session());

  z_owned_subscriber_t state_sub = z_declare_subscriber(
    z_loan(*zenoh_session),
    z_keyexpr("vehicle_gateway/*/state"), z_move(callback), NULL);

  rclcpp::WallRate loop_rate(std::chrono::milliseconds(500));
  while (rclcpp::ok()) {
    loop_rate.sleep();
  }

/*
  const float TARGET_ATTITUDE = 30.0f;
  std::cout << "Arming..." << std::endl;
  vg->gateway_->arm_sync();
  sleep(2);

  const auto home_position = vg->gateway_->get_latlon();
  if (home_position.size() != 3) {
    throw std::runtime_error("home_position should have three elements: lat, lon, alt");
  }
  std::cout << "Home altitude: " << home_position[2] << std::endl;

  std::cout << "Takeoff!" << std::endl;
  vg->gateway_->takeoff();
  sleep(1);

  vg->gateway_->go_to_latlon_sync(
    home_position[0], home_position[1],
    home_position[2] + TARGET_ATTITUDE);

  vg->gateway_->transition_to_fw_sync();

  std::cout << "Begin transitioning to Offboard control..." << std::endl;
  vg->gateway_->transition_to_offboard_sync();

  std::cout << "Enabled position controller" << std::endl;
  auto target_north = 200.0,
    target_east = 0.0;

  std::cout << "Flying to first waypoint..." << std::endl;
  vg->gateway_->offboard_mode_go_to_local_setpoint_sync(
    target_north, target_east, -TARGET_ATTITUDE,
    std::numeric_limits<float>::quiet_NaN(), 15);
  std::cout << "Flying to second waypoint..." << std::endl;
  vg->gateway_->offboard_mode_go_to_local_setpoint_sync(
    -target_north, target_east, -TARGET_ATTITUDE,
    std::numeric_limits<float>::quiet_NaN(), 20);
  std::cout << "Flying home..." << std::endl;
  vg->gateway_->offboard_mode_go_to_local_setpoint_sync(
    0, 0, -TARGET_ATTITUDE,
    std::numeric_limits<float>::quiet_NaN(), 20);

  std::cout << "Switching back to hold mode..." << std::endl;
  vg->gateway_->set_onboard_mode();

  std::cout << "Transitioning to multicopter..." << std::endl;
  vg->gateway_->transition_to_mc_sync();
  std::cout << "VTOL state: " << vg->gateway_->get_vtol_state() << std::endl;
  sleep(1);

  vg->gateway_->go_to_latlon_sync(
    home_position[0], home_position[1],
    home_position[2] + TARGET_ATTITUDE);

  std::cout << "Landing..." << std::endl;
  vg->gateway_->land();

  std::cout << "Disarming..." << std::endl;
  vg->gateway_->disarm_sync();
*/

  vg->destroy();
  std::cout << "Demo complete." << std::endl;

  rclcpp::shutdown();
  return 0;
}
