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
#include "SocketBetaflight.hpp"

#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/time.h>

SocketBetaflight::SocketBetaflight(const std::string & _addr, int _port, bool _isServer)
{
  this->addr = _addr;
  this->isServer = _isServer;
  this->port = _port;
}

int SocketBetaflight::init()
{
  int one = 1;

  if ((this->fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    return -2;
  }

  setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));  // can multi-bind
  fcntl(this->fd, F_SETFL, fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);  // nonblock

  this->isServer = isServer;
  memset(&this->si, 0, sizeof(this->si));
  this->si.sin_family = AF_INET;
  this->si.sin_port = htons(this->port);
  this->port = this->port;

  if (this->addr.empty()) {
    this->si.sin_addr.s_addr = htonl(INADDR_ANY);
  } else {
    this->si.sin_addr.s_addr = inet_addr(this->addr.c_str());
  }

  if (this->isServer) {
    if (bind(this->fd, (const struct sockaddr *)&this->si, sizeof(this->si)) == -1) {
      return -1;
    }
  }
  return 0;
}

int SocketBetaflight::udpSend(const void * data, size_t size)
{
  return sendto(this->fd, data, size, 0, (struct sockaddr *)&this->si, sizeof(this->si));
}

int SocketBetaflight::udpRecv(void * data, size_t size, uint32_t timeout_ms)
{
  fd_set fds;
  struct timeval tv;

  FD_ZERO(&fds);
  FD_SET(this->fd, &fds);

  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000UL;
  int ret_select = select(this->fd + 1, &fds, NULL, NULL, &tv);
  if (ret_select != 1) {
    printf("Error %d %d\n", this->fd + 1, ret_select);
    return -1;
  }

  socklen_t len = sizeof(this->si);
  int ret;
  ret = recvfrom(this->fd, data, size, 0, (struct sockaddr *)&this->si, &len);
  return ret;
}
