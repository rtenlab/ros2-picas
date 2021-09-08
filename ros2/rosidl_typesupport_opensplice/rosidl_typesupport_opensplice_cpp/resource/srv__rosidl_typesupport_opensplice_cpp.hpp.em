// generated from
// rosidl_typesupport_opensplice_cpp/resource/srv__rosidl_typesupport_opensplice_cpp.hpp.em
// generated code does not contain a copyright notice

@# Included from rosidl_typesupport_opensplice_cpp/resource/idl__rosidl_typesupport_opensplice_cpp.hpp.em
@{
TEMPLATE(
    'msg__rosidl_typesupport_opensplice_cpp.hpp.em',
    package_name=package_name, interface_path=interface_path, message=service.request_message,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'msg__rosidl_typesupport_opensplice_cpp.hpp.em',
    package_name=package_name, interface_path=interface_path, message=service.response_message,
    include_directives=include_directives)
}@

@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
include_parts = [package_name] + list(interface_path.parents[0].parts)
include_dir = '/'.join(include_parts)
include_parts.append(convert_camel_case_to_lower_case_underscore(interface_path.stem))
}@
@{
include_base = '/'.join(include_parts)
header_files = [
    include_base +'__struct.hpp',
    include_dir + '/dds_opensplice/ccpp_' + interface_path.stem + '_.h',
    'rosidl_typesupport_cpp/service_type_support.hpp',
    'rosidl_typesupport_interface/macros.h',
    package_name + '/msg/rosidl_typesupport_opensplice_cpp__visibility_control.h',
]
}@
@[for header_file in header_files]@
@[    if header_file in include_directives]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
#include "@(header_file)"
@[end for]@

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_@(package_name)
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_opensplice_cpp,
  @(', '.join(service.namespaced_type.namespaces)),
  @(service.namespaced_type.name))();

#ifdef __cplusplus
}
#endif
