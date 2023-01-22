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

#ifndef VEHICLE_GATEWAY_PYTHON__EXCEPTIONS_HPP_
#define VEHICLE_GATEWAY_PYTHON__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace vehicle_gateway_python
{
class VehicleGatewayError : public std::runtime_error
{
public:
  explicit VehicleGatewayError(const std::string & error_text);

  ~VehicleGatewayError() = default;
};

class NotImplementedError : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

class InvalidHandle : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

}  // namespace vehicle_gateway_python

#endif  // VEHICLE_GATEWAY_PYTHON__EXCEPTIONS_HPP_
