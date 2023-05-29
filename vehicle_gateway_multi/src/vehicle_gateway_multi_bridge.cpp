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

#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <zenoh.h>

#include <chrono>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

using std::string;
using nlohmann::json;
using namespace std::chrono_literals;


class ZenohBridge
{
public:
  ZenohBridge(unsigned int vehicle_id, rclcpp::Node::SharedPtr node, z_owned_session_t * session)
  {
    this->vehicle_id_ = vehicle_id;
    this->node_ = node;
    this->zenoh_key_name_ = "vehicle_gateway/" + std::to_string(vehicle_id) + "/state";
    std::cout << "Declaring Publisher on '" << this->zenoh_key_name_ << "'...\n";
    this->session_ = session;
  }

  std::string zenoh_key()
  {
    return this->zenoh_key_name_;
  }

  int initialize()
  {
    std::string vehicle_id_prefix = std::string("/px4_") +
      std::to_string(this->vehicle_id_);

    rclcpp::QoS qos_profile(10);
    qos_profile
    // Guaranteed delivery is needed to send messages to late-joining subscriptions.
    .best_effort();

    std::cout << "Subscribed to " << vehicle_id_prefix + "/fmu/out/vehicle_gps_position" << '\n';

    subscription_ =
      this->node_->create_subscription<px4_msgs::msg::SensorGps>(
      vehicle_id_prefix + "/fmu/out/vehicle_gps_position",
      qos_profile,
      [this](px4_msgs::msg::SensorGps::ConstSharedPtr msg) {
        {
          const std::lock_guard<std::mutex> lock(mutex_);
          this->lat_ = msg->lat * 1e-7;
          this->lon_ = msg->lon * 1e-7;
          this->alt_ = msg->alt * 1e-3;
          this->timestamp_ = msg->timestamp;
          this->newdata_ = true;
        }
      });

    std::cout << "sending state message...\n";
    this->pub_ = z_declare_publisher(
      z_loan(*this->session_), z_keyexpr(this->zenoh_key_name_.c_str()), NULL);
    if (!z_check(this->pub_)) {
      std::cout << "Unable to declare Publisher for key expression!\n";
      return -1;
    }
    return 0;
  }

  void publish_data()
  {
    const std::lock_guard<std::mutex> lock(mutex_);

    if (!this->newdata_) {
      return;
    }

    json j;
    j["id"] = this->vehicle_id_;
    j["east"] = this->lat_;
    j["north"] = this->lon_;
    j["down"] = -this->alt_;
    j["timestamp"] = this->timestamp_;
    this->newdata_ = false;

    string s = j.dump();
    z_publisher_put_options_t options = z_publisher_put_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_TEXT_JSON, NULL);
    z_publisher_put(
      z_loan(this->pub_),
      reinterpret_cast<const uint8_t *>(s.c_str()),
      s.size() + 1,
      &options);
  }

private:
  std::mutex mutex_;
  double lat_{0};
  double lon_{0};
  double alt_{0};
  uint64_t timestamp_;
  bool newdata_{false};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr subscription_;

  std::string zenoh_key_name_;
  unsigned int vehicle_id_;
  z_owned_session_t * session_;
  z_owned_publisher_t pub_;
};

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cout << "usage: vehicle_gateway_multi_bridge ZENOH_CONFIG_FILENAME VEHICLE_ID\n";
    return EXIT_FAILURE;
  }
  const char * zenoh_config_file = argv[1];
  z_owned_config_t config = zc_config_from_file(zenoh_config_file);
  if (!z_check(config)) {
    std::cout << "unable to parse zenoh config from [" << zenoh_config_file << "]\n";
    return EXIT_FAILURE;
  }
  std::cout << "opening zenoh session...\n";
  z_owned_session_t session = z_open(z_move(config));
  if (!z_check(session)) {
    std::cout << "unable to open zenoh session\n";
    return EXIT_FAILURE;
  }
  std::cout << "zenoh session open!\n";

  int vehicle_id = atoi(argv[2]);

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;
  std::thread spin_thread;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;

  exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  node = std::make_shared<rclcpp::Node>("ZenohBridge");
  exec->add_node(node);
  spin_thread = std::thread(
    [&]() {
      exec->spin();
    });

  std::shared_ptr<ZenohBridge> bridge = std::make_shared<ZenohBridge>(vehicle_id, node, &session);
  bridge->initialize();

  json j;

  rclcpp::WallRate loop_rate(500ms);

  while (rclcpp::ok()) {
    bridge->publish_data();

    loop_rate.sleep();
  }

  rclcpp::shutdown();

  std::cout << "closing zenoh session...\n";
  z_close(z_move(session));

  return EXIT_SUCCESS;
}
