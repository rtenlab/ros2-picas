// generated from rosidl_typesupport_opensplice_c/resource/idl__dds_opensplice__type_support.c.em
// generated code does not contain a copyright notice

#include <cassert>
#include <codecvt>
#include <cstring>
#include <iostream>
#include <limits>
#include <locale>
#include <sstream>

#include <CdrTypeSupport.h>
#include <u_instanceHandle.h>

@{
#######################################################################
# EmPy template for generating <idl>__rosidl_typesupport.c files
#
# Context:
#  - package_name (string)
#  - content (rosidl_parser.definition.IdlContent result of parsing IDL file)
#  - interface_path (Path relative to the directory named after the package)
#######################################################################

include_directives = set()

#######################################################################
# Handle message
#######################################################################
from rosidl_parser.definition import Message
for message in content.get_elements_of_type(Message):
    TEMPLATE(
        'msg__type_support_c.cpp.em',
        package_name=package_name, interface_path=interface_path, message=message,
        include_directives=include_directives)

#######################################################################
# Handle service
#######################################################################
from rosidl_parser.definition import Service
for service in content.get_elements_of_type(Service):
    TEMPLATE(
        'srv__type_support_c.cpp.em',
        package_name=package_name, interface_path=interface_path, service=service,
        include_directives=include_directives)

#######################################################################
# Handle action
#######################################################################
from rosidl_parser.definition import Action
for action in content.get_elements_of_type(Action):
    TEMPLATE(
        'msg__type_support_c.cpp.em',
        package_name=package_name, interface_path=interface_path, message=action.goal,
        include_directives=include_directives)
    TEMPLATE(
        'msg__type_support_c.cpp.em',
        package_name=package_name, interface_path=interface_path, message=action.result,
        include_directives=include_directives)
    TEMPLATE(
        'msg__type_support_c.cpp.em',
        package_name=package_name, interface_path=interface_path, message=action.feedback,
        include_directives=include_directives)
    TEMPLATE(
        'srv__type_support_c.cpp.em',
        package_name=package_name, interface_path=interface_path, service=action.send_goal_service,
        include_directives=include_directives)
    TEMPLATE(
        'srv__type_support_c.cpp.em',
        package_name=package_name, interface_path=interface_path, service=action.get_result_service,
        include_directives=include_directives)
    TEMPLATE(
        'msg__type_support_c.cpp.em',
        package_name=package_name, interface_path=interface_path, message=action.feedback_message,
        include_directives=include_directives)
}@
