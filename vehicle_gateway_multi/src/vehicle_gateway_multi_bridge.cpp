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

#include <cstdio>
#include "zenoh.h"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    printf("usage: vehicle_gateway_multi_bridge CONFIG_FILENAME\n");
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

  printf("closing zenoh session...\n");
  z_close(z_move(session));

  return EXIT_SUCCESS;
}
