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

#include "betaflight_gazebo/BetaflightSocket.hpp"

namespace betaflight_gazebo
{
/// \brief constructor
BetaflightSocket::BetaflightSocket()
{
  // initialize socket udp socket
  fd = socket(AF_INET, SOCK_DGRAM, 0);
  #ifndef _WIN32
  // Windows does not support FD_CLOEXEC
  fcntl(fd, F_SETFD, FD_CLOEXEC);
  #endif
}

/// \brief destructor
BetaflightSocket::~BetaflightSocket()
{
  if (fd != -1) {
    ::close(fd);
    fd = -1;
  }
}

/// \brief Bind to an address and port
/// \param[in] _address Address to bind to.
/// \param[in] _port Port to bind to.
/// \return True on success.
bool BetaflightSocket::Bind(const char * _address, const uint16_t _port)
{
  struct sockaddr_in sockaddr;
  this->MakeSockAddr(_address, _port, sockaddr);

  this->recv = sockaddr;

  if (bind(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
    shutdown(this->fd, 0);
    #ifdef _WIN32
    closesocket(this->fd);
    #else
    close(this->fd);
    #endif
    return false;
  }
  int one = 1;
  setsockopt(
    this->fd, SOL_SOCKET, SO_REUSEADDR,
    reinterpret_cast<const char *>(&one), sizeof(one));

  fcntl(this->fd, F_SETFL, fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
  return true;
}

/// \brief Make a socket
/// \param[in] _address Socket address.
/// \param[in] _port Socket port
/// \param[out] _sockaddr New socket address structure.
void BetaflightSocket::MakeSockAddr(
  const char * _address, const uint16_t _port,
  struct sockaddr_in & _sockaddr)
{
  memset(&_sockaddr, 0, sizeof(_sockaddr));

  #ifdef HAVE_SOCK_SIN_LEN
  _sockaddr.sin_len = sizeof(_sockaddr);
  #endif

  _sockaddr.sin_port = htons(_port);
  _sockaddr.sin_family = AF_INET;
  _sockaddr.sin_addr.s_addr = inet_addr(_address);
}

ssize_t BetaflightSocket::Send(const void * _buf, size_t _size)
{
  return send(this->fd, _buf, _size, 0);
}

bool BetaflightSocket::Connect(const char * _address, const uint16_t _port)
{
  struct sockaddr_in sockaddr;
  this->MakeSockAddr(_address, _port, sockaddr);

  if (connect(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
    shutdown(this->fd, 0);
    close(this->fd);
    return false;
  }
  int one = 1;
  setsockopt(
    this->fd, SOL_SOCKET, SO_REUSEADDR,
    reinterpret_cast<const char *>(&one), sizeof(one));

  fcntl(this->fd, F_SETFL, fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
  return true;
}

/// \brief Receive data
/// \param[out] _buf Buffer that receives the data.
/// \param[in] _size Size of the buffer.
/// \param[in] _timeoutMS Milliseconds to wait for data.
ssize_t BetaflightSocket::Recv(void * _buf, const size_t _size, uint32_t _timeoutMs)
{
  fd_set fds;
  struct timeval tv;

  FD_ZERO(&fds);
  FD_SET(this->fd, &fds);

  tv.tv_sec = _timeoutMs / 1000;
  tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

  if (select(this->fd + 1, &fds, NULL, NULL, &tv) != 1) {
    return -1;
  }

  socklen_t len = sizeof(this->recv);
  return recvfrom(this->fd, _buf, _size, 0, (struct sockaddr *)&this->recv, &len);
}
}  // namespace betaflight_gazebo
