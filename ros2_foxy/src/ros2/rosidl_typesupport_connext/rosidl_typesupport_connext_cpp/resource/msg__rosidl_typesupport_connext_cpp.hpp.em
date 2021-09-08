@# Included from rosidl_typesupport_connext_cpp/resource/idl__rosidl_typesupport_connext_cpp.hpp.em
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
include_parts = [package_name] + list(interface_path.parents[0].parts)
include_base = '/'.join(include_parts)
header_filename = convert_camel_case_to_lower_case_underscore(interface_path.stem)
header_files = [
    'rosidl_runtime_c/message_type_support_struct.h',
    'rosidl_typesupport_interface/macros.h',
    package_name + '/msg/rosidl_typesupport_connext_cpp__visibility_control.h',
    include_base + '/detail/' + header_filename + '__struct.hpp'
]
dds_specific_header_files = [
    include_base + '/dds_connext/' + interface_path.stem + '_Support.h',
    include_base + '/dds_connext/' + interface_path.stem + '_Plugin.h',
    'ndds/ndds_cpp.h'
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

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif

@[for header_file in dds_specific_header_files]@
@[    if header_file in include_directives]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
#include "@(header_file)"
@[end for]@

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// forward declaration of internal CDR Stream
struct ConnextStaticCDRStream;

// forward declaration of DDS types
class DDSDomainParticipant;
class DDSDataWriter;
class DDSDataReader;

@[for ns in message.structure.namespaced_type.namespaces]@

namespace @(ns)
{
@[end for]@
@{
__ros_msg_pkg_prefix = '::'.join(message.structure.namespaced_type.namespaces)
__ros_msg_type = __ros_msg_pkg_prefix + '::' + message.structure.namespaced_type.name
__dds_msg_type_prefix = __ros_msg_pkg_prefix + '::dds_::' + message.structure.namespaced_type.name
__dds_msg_type = __dds_msg_type_prefix + '_'
}@
namespace typesupport_connext_cpp
{

DDS_TypeCode *
get_type_code__@(message.structure.namespaced_type.name)();

bool
ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC_@(package_name)
convert_ros_message_to_dds(
  const @(__ros_msg_type) & ros_message,
  @(__dds_msg_type) & dds_message);

bool
ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC_@(package_name)
convert_dds_message_to_ros(
  const @(__dds_msg_type) & dds_message,
  @(__ros_msg_type) & ros_message);

bool
to_cdr_stream__@(message.structure.namespaced_type.name)(
  const void * untyped_ros_message,
  ConnextStaticCDRStream * cdr_stream);

bool
to_message__@(message.structure.namespaced_type.name)(
  const ConnextStaticCDRStream * cdr_stream,
  void * untyped_ros_message);

}  // namespace typesupport_connext_cpp

@[for ns in reversed(message.structure.namespaced_type.namespaces)]@
}  // namespace @(ns)

@[end for]@

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC_@(package_name)
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_cpp,
  @(', '.join([package_name] + list(interface_path.parents[0].parts))),
  @(message.structure.namespaced_type.name))();

#ifdef __cplusplus
}
#endif

