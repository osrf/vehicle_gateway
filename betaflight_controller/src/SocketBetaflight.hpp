/**
 * Copyright (c) 2017 cs8425
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license.
 */

#ifndef SOCKETBETAFLIGHT_HPP
#define SOCKETBETAFLIGHT_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>

class SocketBetaflight
{
public:
  SocketBetaflight(){};
  SocketBetaflight(const std::string &addr, int port, bool isServer);

  int init();
  int udpRecv(void* data, size_t size, uint32_t timeout_ms);
  int udpSend(const void* data, size_t size);

private:
    int fd;
    struct sockaddr_in si;
    int port;
    std::string addr;
    bool isServer;
};

#endif
