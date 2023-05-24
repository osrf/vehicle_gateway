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

struct gpsdata
{
  double lat{0};
  double lon{0};
  double alt{0};
  uint64_t timestamp;
};

std::mutex mutex_positions;
std::unordered_map<int, gpsdata> map_positions;
std::vector<rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr> subscriptions;

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cout << "usage: vehicle_gateway_multi_bridge ZENOH_CONFIG_FILENAME VEHICLE_ID\n";
    return EXIT_FAILURE;
  }
  const char * config_filename = argv[1];
  z_owned_config_t config = zc_config_from_file(config_filename);
  if (!z_check(config)) {
    std::cout << "unable to parse zenoh config from [" << config_filename << "]\n";
    return EXIT_FAILURE;
  }
  std::cout << "opening zenoh session...\n";
  z_owned_session_t session = z_open(z_move(config));
  if (!z_check(session)) {
    std::cout << "unable to open zenoh session\n";
    return EXIT_FAILURE;
  }
  std::cout << "zenoh session open!\n";

  YAML::Node config_yaml = YAML::LoadFile(argv[2]);

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

  rclcpp::QoS qos_profile(10);
  qos_profile
  // Guaranteed delivery is needed to send messages to late-joining subscriptions.
  .best_effort();

  for (std::size_t i = 0; i < config_yaml.size(); i++) {
    std::string vehicle_id_prefix = std::string("/px4_") +
      config_yaml[i]["vehicle_id"].as<std::string>();
    int vehicle_id = config_yaml[i]["vehicle_id"].as<int>();

    std::cout << "Subscribed to " << vehicle_id_prefix + "/fmu/out/vehicle_gps_position" << '\n';

    subscriptions.push_back(
      node->create_subscription<px4_msgs::msg::SensorGps>(
        vehicle_id_prefix + "/fmu/out/vehicle_gps_position",
        qos_profile,
        [&, vehicle_id](px4_msgs::msg::SensorGps::ConstSharedPtr msg) {
          std::cerr << "vehicle_id " << vehicle_id << '\n';
          struct gpsdata data{msg->lat * 1e-7, msg->lon * 1e-7, msg->alt * 1e-3,
            msg->timestamp};
          mutex_positions.lock();
          map_positions[vehicle_id] = data;
          mutex_positions.unlock();
        }));
  }

  const char * keyexpr = "vehicle_gateway/positions";

  std::cout << "Declaring Publisher on '" << keyexpr << "'...\n";
  z_owned_publisher_t pub = z_declare_publisher(z_loan(session), z_keyexpr(keyexpr), NULL);
  if (!z_check(pub)) {
    std::cout << "Unable to declare Publisher for key expression!\n";
    exit(-1);
  }

  json j;

  rclcpp::WallRate loop_rate(500ms);

  while (rclcpp::ok()) {
    std::cout << "sending telemetry message...\n";

    mutex.lock();
    for (auto it = map_positions.begin(); it != map_positions.end(); ++it) {
      j["vehicle_" + std::to_string(it->first)]["id"] = it->first;
      j["vehicle_" + std::to_string(it->first)]["east"] = it->second.lat;
      j["vehicle_" + std::to_string(it->first)]["north"] = it->second.lon;
      j["vehicle_" + std::to_string(it->first)]["down"] = -it->second.alt;
      j["vehicle_" + std::to_string(it->first)]["timestamp"] = it->second.timestamp;
    }
    mutex.unlock();
    // todo: send string message via zenoh
    string s = j.dump();

    z_publisher_put_options_t options = z_publisher_put_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_TEXT_PLAIN, NULL);
    std::cerr << "sizeof(s.c_str()) " << sizeof(s.c_str()) << '\n';
    z_publisher_put(z_loan(pub), (const uint8_t *)s.c_str(), s.size() + 1, &options);

    loop_rate.sleep();
  }

  subscriptions.clear();
  rclcpp::shutdown();

  std::cout << "closing zenoh session...\n";
  z_close(z_move(session));

  return EXIT_SUCCESS;
}
