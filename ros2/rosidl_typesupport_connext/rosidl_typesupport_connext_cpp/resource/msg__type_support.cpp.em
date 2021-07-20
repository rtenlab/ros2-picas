@# Included from rosidl_typesupport_connext_cpp/resource/idl__dds_connext__type_support.cpp.em
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import NamespacedType
include_parts = [package_name] + list(interface_path.parents[0].parts)
include_base = '/'.join(include_parts)

include_prefix = convert_camel_case_to_lower_case_underscore(interface_path.stem)

header_files = [
    include_base + '/' + include_prefix + '__rosidl_typesupport_connext_cpp.hpp',
    'rcutils/types/uint8_array.h',
    'rosidl_typesupport_cpp/message_type_support.hpp',
    'rosidl_typesupport_connext_cpp/identifier.hpp',
    'rosidl_typesupport_connext_cpp/message_type_support.h',
    'rosidl_typesupport_connext_cpp/message_type_support_decl.hpp',
    'rosidl_typesupport_connext_cpp/wstring_conversion.hpp',
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

// forward declaration of message dependencies and their conversion functions
@[for member in message.structure.members]@
@{
type_ = member.type
if isinstance(type_, AbstractNestedType):
   type_ = type_.value_type
}@
@[    if isinstance(type_, NamespacedType)]@
@[        for ns in type_.namespaces]@
namespace @(ns)
{
@[        end for]@
namespace dds_
{
class @(type_.name)_;
}  // namespace dds_

namespace typesupport_connext_cpp
{
@{
member_ros_msg_pkg_prefix = '::'.join(type_.namespaces)
member_ros_msg_type = member_ros_msg_pkg_prefix + '::' + type_.name
member_dds_msg_type = member_ros_msg_pkg_prefix + '::dds_::' + type_.name + '_'
}@

bool convert_ros_message_to_dds(
  const @(member_ros_msg_type) &,
  @(member_dds_msg_type) &);
bool convert_dds_message_to_ros(
  const @(member_dds_msg_type) &,
  @(member_ros_msg_type) &);
}  // namespace typesupport_connext_cpp
@[        for ns in reversed(type_.namespaces)]@
}  // namespace @(ns)
@[        end for]@
@[    end if]@
@[end for]@

@[for ns in message.structure.namespaced_type.namespaces]@

namespace @(ns)
{
@[end for]@

namespace typesupport_connext_cpp
{

@{
__ros_msg_pkg_prefix = '::'.join(message.structure.namespaced_type.namespaces)
__ros_msg_type = __ros_msg_pkg_prefix + '::' + message.structure.namespaced_type.name
__dds_msg_type_prefix = __ros_msg_pkg_prefix + '::dds_::' + message.structure.namespaced_type.name
__dds_msg_type = __dds_msg_type_prefix + '_'
}@

DDS_TypeCode *
get_type_code__@(message.structure.namespaced_type.name)()
{
  return @(__dds_msg_type_prefix)_TypeSupport::get_typecode();
}

bool
convert_ros_message_to_dds(
  const @(__ros_msg_type) & ros_message,
  @(__dds_msg_type) & dds_message)
{
@[if not message.structure.members]@
  (void)ros_message;
  (void)dds_message;
@[end if]@
@[for member in message.structure.members]@
  // member.name @(member.name)
@[  if isinstance(member.type, AbstractNestedType)]@
  {
@[    if isinstance(member.type, Array)]@
    size_t size = @(member.type.size);
@[    else]@
    size_t size = ros_message.@(member.name).size();
    if (size > (std::numeric_limits<DDS_Long>::max)()) {
      throw std::runtime_error("array size exceeds maximum DDS sequence size");
    }
@[      if isinstance(member.type, BoundedSequence)]@
    if (size > @(member.type.maximum_size)) {
      throw std::runtime_error("array size exceeds upper bound");
    }
@[      end if]@
    DDS_Long length = static_cast<DDS_Long>(size);
    if (length > dds_message.@(member.name)_.maximum()) {
      if (!dds_message.@(member.name)_.maximum(length)) {
        throw std::runtime_error("failed to set maximum of sequence");
      }
    }
    if (!dds_message.@(member.name)_.length(length)) {
      throw std::runtime_error("failed to set length of sequence");
    }
@[    end if]@
    for (size_t i = 0; i < size; i++) {
@[    if isinstance(member.type.value_type, AbstractString)]@
      DDS_String_free(dds_message.@(member.name)_[static_cast<DDS_Long>(i)]);
      dds_message.@(member.name)_[static_cast<DDS_Long>(i)] =
        DDS_String_dup(ros_message.@(member.name)[i].c_str());
@[    elif isinstance(member.type.value_type, AbstractWString)]@
      DDS_Wchar * wstr = rosidl_typesupport_connext_cpp::create_wstring_from_u16string(ros_message.@(member.name)[i]);
      if (NULL == wstr) {
        fprintf(stderr, "failed to create wstring from u16string\n");
        return false;
      }
      DDS_Wstring_free(dds_message.@(member.name)_[static_cast<DDS_Long>(i)]);
      dds_message.@(member.name)_[static_cast<DDS_Long>(i)] = wstr;
@[    elif isinstance(member.type.value_type, BasicType)]@
      dds_message.@(member.name)_[static_cast<DDS_Long>(i)] =
        ros_message.@(member.name)[i];
@[    else]@
      if (
        !@('::'.join(member.type.value_type.namespaces))::typesupport_connext_cpp::convert_ros_message_to_dds(
          ros_message.@(member.name)[i],
          dds_message.@(member.name)_[static_cast<DDS_Long>(i)]))
      {
        return false;
      }
@[    end if]@
    }
  }
@[  elif isinstance(member.type, AbstractString)]@
  DDS_String_free(dds_message.@(member.name)_);
  dds_message.@(member.name)_ =
    DDS_String_dup(ros_message.@(member.name).c_str());
@[  elif isinstance(member.type, AbstractWString)]@
  {
    DDS_Wchar * wstr = rosidl_typesupport_connext_cpp::create_wstring_from_u16string(ros_message.@(member.name));
    if (NULL == wstr) {
      fprintf(stderr, "failed to create wstring from u16string\n");
      return false;
    }
    DDS_Wstring_free(dds_message.@(member.name)_);
    dds_message.@(member.name)_ = wstr;
  }
@[  elif isinstance(member.type, BasicType)]@
  dds_message.@(member.name)_ =
    ros_message.@(member.name);
@[  else]@
  if (
    !@('::'.join(member.type.namespaces))::typesupport_connext_cpp::convert_ros_message_to_dds(
      ros_message.@(member.name),
      dds_message.@(member.name)_))
  {
    return false;
  }
@[  end if]@

@[end for]@
  return true;
}

bool
convert_dds_message_to_ros(
  const @(__dds_msg_type) & dds_message,
  @(__ros_msg_type) & ros_message)
{
@[if not message.structure.members]@
  (void)ros_message;
  (void)dds_message;
@[end if]@
@[for member in message.structure.members]@
  // member.name @(member.name)
@[  if isinstance(member.type, AbstractNestedType)]@
  {
@[    if isinstance(member.type, Array)]@
    size_t size = @(member.type.size);
@[    else]@
    size_t size = dds_message.@(member.name)_.length();
    ros_message.@(member.name).resize(size);
@[    end if]@
    for (size_t i = 0; i < size; i++) {
@[    if isinstance(member.type.value_type, BasicType)]@
      ros_message.@(member.name)[i] =
        dds_message.@(member.name)_[static_cast<DDS_Long>(i)]@(' == DDS_BOOLEAN_TRUE' if member.type.value_type.typename == 'boolean' else '');
@[    elif isinstance(member.type.value_type, AbstractString)]@
      ros_message.@(member.name)[i] =
        dds_message.@(member.name)_[static_cast<DDS_Long>(i)];
@[    elif isinstance(member.type.value_type, AbstractWString)]@
      bool succeeded = rosidl_typesupport_connext_cpp::wstring_to_u16string(dds_message.@(member.name)_[static_cast<DDS_Long>(i)], ros_message.@(member.name)[i]);
      if (!succeeded) {
        fprintf(stderr, "failed to create wstring from u16string\n");
        return false;
      }
@[    else]@
      if (
        !@('::'.join(member.type.value_type.namespaces))::typesupport_connext_cpp::convert_dds_message_to_ros(
          dds_message.@(member.name)_[static_cast<DDS_Long>(i)],
          ros_message.@(member.name)[i]))
      {
        return false;
      }
@[    end if]@
    }
  }
@[  elif isinstance(member.type, BasicType)]@
  ros_message.@(member.name) =
    dds_message.@(member.name)_@(' == DDS_BOOLEAN_TRUE' if member.type.typename == 'boolean' else '');
@[  elif isinstance(member.type, AbstractString)]@
  ros_message.@(member.name) = dds_message.@(member.name)_;
@[  elif isinstance(member.type, AbstractWString)]@
  {
    bool succeeded = rosidl_typesupport_connext_cpp::wstring_to_u16string(dds_message.@(member.name)_, ros_message.@(member.name));
    if (!succeeded) {
      fprintf(stderr, "failed to create wstring from u16string\n");
      return false;
    }
  }
@[  else]@
  if (
    !@('::'.join(member.type.namespaces))::typesupport_connext_cpp::convert_dds_message_to_ros(
      dds_message.@(member.name)_,
      ros_message.@(member.name)))
  {
    return false;
  }
@[  end if]@

@[end for]@
  return true;
}

bool
to_cdr_stream__@(message.structure.namespaced_type.name)(
  const void * untyped_ros_message,
  rcutils_uint8_array_t * cdr_stream)
{
  if (!cdr_stream) {
    return false;
  }
  if (!untyped_ros_message) {
    return false;
  }

  // cast the untyped to the known ros message
  const @(__ros_msg_type) & ros_message =
    *(const @(__ros_msg_type) *)untyped_ros_message;

  // create a respective connext dds type
  @(__dds_msg_type) * dds_message = @(__dds_msg_type_prefix)_TypeSupport::create_data();
  if (!dds_message) {
    return false;
  }

  // convert ros to dds
  if (!convert_ros_message_to_dds(ros_message, *dds_message)) {
    return false;
  }

  // call the serialize function for the first time to get the expected length of the message
  unsigned int expected_length;
  if (@(__dds_msg_type_prefix)_Plugin_serialize_to_cdr_buffer(
      NULL,
      &expected_length,
      dds_message) != RTI_TRUE)
  {
    fprintf(stderr, "failed to call @(__dds_msg_type_prefix)_Plugin_serialize_to_cdr_buffer()\n");
    return false;
  }
  cdr_stream->buffer_length = expected_length;
  if (cdr_stream->buffer_length > (std::numeric_limits<unsigned int>::max)()) {
    fprintf(stderr, "cdr_stream->buffer_length, unexpectedly larger than max unsigned int\n");
    return false;
  }
  if (cdr_stream->buffer_capacity < cdr_stream->buffer_length) {
    cdr_stream->allocator.deallocate(cdr_stream->buffer, cdr_stream->allocator.state);
    cdr_stream->buffer = static_cast<uint8_t *>(cdr_stream->allocator.allocate(cdr_stream->buffer_length, cdr_stream->allocator.state));
  }
  // call the function again and fill the buffer this time
  unsigned int buffer_length_uint = static_cast<unsigned int>(cdr_stream->buffer_length);
  if (@(__dds_msg_type_prefix)_Plugin_serialize_to_cdr_buffer(
      reinterpret_cast<char *>(cdr_stream->buffer),
      &buffer_length_uint,
      dds_message) != RTI_TRUE)
  {
    return false;
  }
  if (@(__dds_msg_type_prefix)_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return true;
}

bool
to_message__@(message.structure.namespaced_type.name)(
  const rcutils_uint8_array_t * cdr_stream,
  void * untyped_ros_message)
{
  if (!cdr_stream) {
    return false;
  }
  if (!cdr_stream->buffer) {
    fprintf(stderr, "cdr stream doesn't contain data\n");
  }
  if (!untyped_ros_message) {
    return false;
  }

  @(__dds_msg_type) * dds_message =
    @(__dds_msg_type_prefix)_TypeSupport::create_data();
  if (cdr_stream->buffer_length > (std::numeric_limits<unsigned int>::max)()) {
    fprintf(stderr, "cdr_stream->buffer_length, unexpectedly larger than max unsigned int\n");
    return false;
  }
  if (@(__dds_msg_type_prefix)_Plugin_deserialize_from_cdr_buffer(
      dds_message,
      reinterpret_cast<char *>(cdr_stream->buffer),
      static_cast<unsigned int>(cdr_stream->buffer_length)) != RTI_TRUE)
  {
    fprintf(stderr, "deserialize from cdr buffer failed\n");
    return false;
  }

  @(__ros_msg_type) & ros_message =
    *(@(__ros_msg_type) *)untyped_ros_message;
  bool success = convert_dds_message_to_ros(*dds_message, ros_message);
  if (@(__dds_msg_type_prefix)_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return success;
}

static message_type_support_callbacks_t _@(message.structure.namespaced_type.name)__callbacks = {
  "@('::'.join([package_name] + list(interface_path.parents[0].parts)))",
  "@(message.structure.namespaced_type.name)",
  &get_type_code__@(message.structure.namespaced_type.name),
  nullptr,
  nullptr,
  &to_cdr_stream__@(message.structure.namespaced_type.name),
  &to_message__@(message.structure.namespaced_type.name)
};

static rosidl_message_type_support_t _@(message.structure.namespaced_type.name)__handle = {
  rosidl_typesupport_connext_cpp::typesupport_identifier,
  &_@(message.structure.namespaced_type.name)__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_connext_cpp

@[for ns in reversed(message.structure.namespaced_type.namespaces)]@
}  // namespace @(ns)

@[end for]@

namespace rosidl_typesupport_connext_cpp
{

template<>
ROSIDL_TYPESUPPORT_CONNEXT_CPP_EXPORT_@(package_name)
const rosidl_message_type_support_t *
get_message_type_support_handle<@(__ros_msg_type)>()
{
  return &@(__ros_msg_pkg_prefix)::typesupport_connext_cpp::_@(message.structure.namespaced_type.name)__handle;
}

}  // namespace rosidl_typesupport_connext_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_cpp,
  @(', '.join([package_name] + list(interface_path.parents[0].parts))),
  @(message.structure.namespaced_type.name))()
{
  return &@(__ros_msg_pkg_prefix)::typesupport_connext_cpp::_@(message.structure.namespaced_type.name)__handle;
}

#ifdef __cplusplus
}
#endif
