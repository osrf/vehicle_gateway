#include <cstdio>
#include "zenoh.h"

int main(int argc, char ** argv)
{
  printf("hello world\n");
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
