# Copyright 2017-2019 Open Source Robotics Foundation, Inc.
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

import array

from typing import Any
from typing import Dict

import numpy

from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import NamespacedType
from rosidl_runtime_py.convert import get_message_slot_types
from rosidl_runtime_py.import_message import import_message_from_namespaced_type


def set_message_fields(msg: Any, values: Dict[str, str]) -> None:
    """
    Set the fields of a ROS message.

    :param msg: The ROS message to populate.
    :param values: The values to set in the ROS message. The keys of the dictionary represent
        fields of the message.
    :raises AttributeError: If the message does not have a field provided in the input dictionary.
    :raises TypeError: If a message value does not match its field type.
    """
    try:
        items = values.items()
    except AttributeError:
        raise TypeError(
            "Value '%s' is expected to be a dictionary but is a %s" %
            (values, type(values).__name__))
    for field_name, field_value in items:
        field = getattr(msg, field_name)
        field_type = type(field)
        if field_type is array.array:
            value = field_type(field.typecode, field_value)
        elif field_type is numpy.ndarray:
            value = numpy.array(field_value, dtype=field.dtype)
        elif type(field_value) is field_type:
            value = field_value
        else:
            try:
                value = field_type(field_value)
            except TypeError:
                value = field_type()
                set_message_fields(value, field_value)
        rosidl_type = get_message_slot_types(msg)[field_name]
        # Check if field is an array of ROS messages
        if isinstance(rosidl_type, AbstractNestedType):
            if isinstance(rosidl_type.value_type, NamespacedType):
                field_elem_type = import_message_from_namespaced_type(rosidl_type.value_type)
                for n in range(len(value)):
                    submsg = field_elem_type()
                    set_message_fields(submsg, value[n])
                    value[n] = submsg
        setattr(msg, field_name, value)
