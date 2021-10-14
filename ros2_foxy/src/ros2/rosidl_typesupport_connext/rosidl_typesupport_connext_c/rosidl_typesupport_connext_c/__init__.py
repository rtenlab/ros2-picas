# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from rosidl_cmake import generate_files


def generate_typesupport_connext_c(arguments_file):
    mapping = {
        'idl__rosidl_typesupport_connext_c.h.em':
        '%s__rosidl_typesupport_connext_c.h',
        'idl__dds_connext__type_support_c.cpp.em':
        'dds_connext/%s__type_support_c.cpp'
    }
    generate_files(arguments_file, mapping)
    return 0
