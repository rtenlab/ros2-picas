@# Included from rosidl_typesupport_connext_c/resource/idl__rosidl_typesupport_connext_c.h.em
@{
header_files = [
    'rosidl_generator_c/message_type_support_struct.h',
    'rosidl_typesupport_interface/macros.h',
    package_name + '/msg/rosidl_typesupport_connext_c__visibility_control.h'
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

ROSIDL_TYPESUPPORT_CONNEXT_C_PUBLIC_@(package_name)
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_c,
  @(', '.join([package_name] + list(interface_path.parents[0].parts))),
  @(message.structure.namespaced_type.name))();

#ifdef __cplusplus
}
#endif
