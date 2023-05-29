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
#include <zenoh.h>
#include <cstdio>

const char * kind_to_str(z_sample_kind_t kind);

void data_handler(const z_sample_t * sample, void * /*arg*/)
{
  z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
  printf(
    ">> [Subscriber] Received %s ('%s': '%.*s')\n", kind_to_str(sample->kind), z_loan(keystr),
    static_cast<int>(sample->payload.len), sample->payload.start);
  z_drop(z_move(keystr));
}

int main(int argc, char ** argv)
{
  const char * expr = "vehicle_gateway/*/state";
  if (argc > 1) {
    expr = argv[1];
  }

  z_owned_config_t config = z_config_default();
  if (argc > 2) {
    if (zc_config_insert_json(z_loan(config), Z_CONFIG_LISTEN_KEY, argv[2]) < 0) {
      printf(
        "Couldn't insert value `%s` in configuration at `%s`. "
        "This is likely because `%s` expects a "
        "JSON-serialized list of strings\n",
        argv[2], Z_CONFIG_LISTEN_KEY,
        Z_CONFIG_LISTEN_KEY);
      exit(-1);
    }
  }

  printf("Opening session...\n");
  z_owned_session_t s = z_open(z_move(config));
  if (!z_check(s)) {
    printf("Unable to open session!\n");
    exit(-1);
  }

  z_owned_closure_sample_t callback = z_closure(data_handler);
  printf("Declaring Subscriber on '%s'...\n", expr);
  z_owned_subscriber_t sub =
    z_declare_subscriber(z_loan(s), z_keyexpr(expr), z_move(callback), NULL);
  if (!z_check(sub)) {
    printf("Unable to declare subscriber.\n");
    exit(-1);
  }

  printf("Enter 'q' to quit...\n");
  char c = 0;
  while (c != 'q') {
    c = getchar();
    if (c == -1) {
      sleep(1);
    }
  }

  z_undeclare_subscriber(z_move(sub));
  z_close(z_move(s));
  return 0;
}

const char * kind_to_str(z_sample_kind_t kind)
{
  switch (kind) {
    case Z_SAMPLE_KIND_PUT:
      return "PUT";
    case Z_SAMPLE_KIND_DELETE:
      return "DELETE";
    default:
      return "UNKNOWN";
  }
}
