# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import importlib
import os


def _import(name):
    try:
        return importlib.import_module(name, package='rosbag2_transport')
    except ImportError as e:
        if e.path is not None and os.path.isfile(e.path):
            e.msg += \
                "\nThe C extension '%s' failed to be imported while being present on the system." \
                " Please refer to '%s' for possible solutions" % \
                (e.path, 'https://index.ros.org/doc/ros2/Troubleshooting/'
                         'Installation-Troubleshooting/'
                         '#import-failing-without-library-present-on-the-system')
        raise


rosbag2_transport_py = _import('._rosbag2_transport_py')
