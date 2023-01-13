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

#ifndef SOCKETBETAFLIGHT_HPP_
#define SOCKETBETAFLIGHT_HPP_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>

class SocketBetaflight
{
public:
  SocketBetaflight() {}
  SocketBetaflight(const std::string & addr, int port, bool isServer);

  int init();
  int udpRecv(void * data, size_t size, uint32_t timeout_ms);
  int udpSend(const void * data, size_t size);

private:
  int fd;
  struct sockaddr_in si;
  int port;
  std::string addr;
  bool isServer;
};

#endif  // SOCKETBETAFLIGHT_HPP_
