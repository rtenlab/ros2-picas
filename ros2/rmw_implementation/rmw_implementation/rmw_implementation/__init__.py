# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Utilities to discover available RMW implementations."""

import ament_index_python


def get_available_rmw_implementations():
    """Return the set of all available RMW implementations as registered in the ament index."""
    rmw_implementations = ament_index_python.get_resources('rmw_typesupport')
    return {name for name in rmw_implementations if name != 'rmw_implementation'}


__all__ = [
    'get_available_rmw_implementations',
]
