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

"""
Provide singleton access to the vehicle_gateway_python C modules.

For example, you might use it like this:

.. code::

    from vehicle_gateway_python.impl.implementation_singleton \
      import vehicle_gateway_python_implementation as _vehicle_gateway

    _vehicle_gateway_python.init()
"""

from rpyutils import import_c_library
package = 'vehicle_gateway'

vehicle_gateway_implementation = import_c_library('._vehicle_gateway_pybind11', package)
