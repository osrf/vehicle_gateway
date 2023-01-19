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

#ifndef BETAFLIGHT_GAZEBO__BETAFLIGHTSOCKET_HPP_
#define BETAFLIGHT_GAZEBO__BETAFLIGHTSOCKET_HPP_

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <cstring>
#include <string>

namespace betaflight_gazebo
{
// Private data class
class BetaflightSocket
{
  /// \brief constructor

public:
  BetaflightSocket();

  /// \brief destructor

public:
  ~BetaflightSocket();

  /// \brief Bind to an address and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.

public:
  bool Bind(const char * _address, const uint16_t _port);

  /// \brief Connect to an address and port
  /// \param[in] _address Address to connect to.
  /// \param[in] _port Port to connect to.
  /// \return True on success.

public:
  bool Connect(const char * _address, const uint16_t _port);

  /// \brief Make a socket
  /// \param[in] _address Socket address.
  /// \param[in] _port Socket port
  /// \param[out] _sockaddr New socket address structure.

public:
  void MakeSockAddr(
    const char * _address, const uint16_t _port,
    struct sockaddr_in & _sockaddr);

public:
  ssize_t Send(const void * _buf, size_t _size);

  /// \brief Receive data
  /// \param[out] _buf Buffer that receives the data.
  /// \param[in] _size Size of the buffer.
  /// \param[in] _timeoutMS Milliseconds to wait for data.

public:
  ssize_t Recv(void * _buf, const size_t _size, uint32_t _timeoutMs);

  /// \brief Socket handle

private:
  int fd;
  struct sockaddr_in recv;
};
}  // namespace betaflight_gazebo

#endif  // BETAFLIGHT_GAZEBO__BETAFLIGHTSOCKET_HPP_
