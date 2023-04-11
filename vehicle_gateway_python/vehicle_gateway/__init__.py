# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List, Optional

from vehicle_gateway.impl.implementation_singleton import vehicle_gateway_implementation \
  as _vehicle_gateway

ArmingState = _vehicle_gateway.ArmingState
ArmDisarmReason = _vehicle_gateway.ArmDisarmReason
FlightMode = _vehicle_gateway.FlightMode
Failure = _vehicle_gateway.Failure
VehicleType = _vehicle_gateway.VehicleType
ControllerType = _vehicle_gateway.ControllerType
VtolState = _vehicle_gateway.VtolState


def init(
    args: Optional[List[str]] = None,
    plugin_type: str = None,
) -> None:
    args if args is None else []
    return _vehicle_gateway.VehicleGatewayPython(args, plugin_type)
