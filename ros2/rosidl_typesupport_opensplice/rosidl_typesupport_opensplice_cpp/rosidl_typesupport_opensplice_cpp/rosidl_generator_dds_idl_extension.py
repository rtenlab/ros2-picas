# Copyright 2014 Open Source Robotics Foundation, Inc.
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

from rosidl_generator_dds_idl import get_post_struct_lines as \
    get_default_post_struct_lines
from rosidl_generator_dds_idl import idl_typename as \
    default_idl_typename
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import BasicType


def idl_typename(type_):
    typename = default_idl_typename(type_)
    # OpenSplice doesn't support wchar / wstring, map to char / string instead
    if isinstance(type_, BasicType) and type_.typename == 'wchar':
        typename == 'char'
    if isinstance(type_, AbstractWString):
        assert typename.startswith('wstring')
        typename = typename[1:]
    return typename


def get_post_struct_lines(message):
    lines = get_default_post_struct_lines(message)
    lines.append('#pragma keylist %s_' % message.structure.namespaced_type.name)
    return lines
