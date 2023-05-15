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

#include <cstdio>
#include <string>

#include "nlohmann/json.hpp"
#include "zenoh.h"

using std::string;
using nlohmann::json;

int main(int argc, char ** argv)
{
  if (argc < 2) {
    printf("usage: vehicle_gateway_multi_bridge ZENOH_CONFIG_FILENAME VEHICLE_ID\n");
    return EXIT_FAILURE;
  }
  const char * config_filename = argv[1];
  z_owned_config_t config = zc_config_from_file(config_filename);
  if (!z_check(config)) {
    printf("unable to parse zenoh config from [%s]\n", config_filename);
    return EXIT_FAILURE;
  }
  printf("opening zenoh session...\n");
  z_owned_session_t session = z_open(z_move(config));
  if (!z_check(session)) {
    printf("unable to open zenoh session\n");
    return EXIT_FAILURE;
  }
  printf("zenoh session open!\n");

  // todo: construct subscribers to vehicle position ROS 2 topic

  json j;

  // todo: exit loop as signaled by ROS 2
  while (true) {
    printf("sending telemetry message...\n");
    // time of telemetry data
    j["sec"] = 42;
    j["nsec"] = 43;
    // telemetry: north/east/down position, add other stuff in future?
    j["east"] = 1.234;
    j["north"] = 2.345;
    j["down"] = -3.456;

    // todo: send string message via zenoh
    string s = j.dump();

    usleep(1000000);  // todo: use ROS 2 timer for pacing
  }

  printf("closing zenoh session...\n");
  z_close(z_move(session));

  return EXIT_SUCCESS;
}
