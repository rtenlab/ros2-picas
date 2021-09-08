@# Included from rosidl_typesupport_connext_c/resource/idl__dds_connext__type_support_c.cpp.em
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_generator_c import idl_structure_type_to_c_include_prefix
from rosidl_generator_c import idl_structure_type_to_c_typename
from rosidl_generator_c import idl_type_to_c
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import NamespacedType
include_parts = [package_name] + list(interface_path.parents[0].parts)
include_base = '/'.join(include_parts)

cpp_include_prefix = interface_path.stem
c_include_prefix = convert_camel_case_to_lower_case_underscore(cpp_include_prefix)

header_files = [
    include_base + '/' + c_include_prefix + '__rosidl_typesupport_connext_c.h',
    'rcutils/types/uint8_array.h',
    'rosidl_typesupport_connext_c/identifier.h',
    'rosidl_typesupport_connext_c/wstring_conversion.hpp',
    'rosidl_typesupport_connext_cpp/message_type_support.h',
    package_name + '/msg/rosidl_typesupport_connext_c__visibility_control.h',
    include_base + '/' + c_include_prefix + '__struct.h',
    include_base + '/' + c_include_prefix + '__functions.h',
]

dds_specific_header_files = [
    include_base + '/dds_connext/' + cpp_include_prefix + '_Support.h',
    include_base + '/dds_connext/' + cpp_include_prefix + '_Plugin.h'
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

// includes and forward declarations of message dependencies and their conversion functions
@# // Include the message header for each non-primitive field.
#if defined(__cplusplus)
extern "C"
{
#endif

@{
from collections import OrderedDict
includes = OrderedDict()
for member in message.structure.members:
    if isinstance(member.type, AbstractSequence) and isinstance(member.type.value_type, BasicType):
       includes.setdefault('rosidl_generator_c/primitives_sequence.h', []).append(member.name)
       includes.setdefault('rosidl_generator_c/primitives_sequence_functions.h', []).append(member.name)
       continue
    type_ = member.type
    if isinstance(type_, AbstractNestedType):
       type_ = type_.value_type
    if isinstance(type_, AbstractString):
        includes.setdefault('rosidl_generator_c/string.h', []).append(member.name)
        includes.setdefault('rosidl_generator_c/string_functions.h', []).append(member.name)
    if isinstance(type_, AbstractWString):
        includes.setdefault('rosidl_generator_c/u16string.h', []).append(member.name)
        includes.setdefault('rosidl_generator_c/u16string_functions.h', []).append(member.name)
    if isinstance(type_, NamespacedType):
        include_prefix = idl_structure_type_to_c_include_prefix(type_)
        if include_prefix.endswith('__request'):
            include_prefix = include_prefix[:-9]
        elif include_prefix.endswith('__response'):
            include_prefix = include_prefix[:-10]
        if include_prefix.endswith('__goal'):
            include_prefix = include_prefix[:-6]
        elif include_prefix.endswith('__result'):
            include_prefix = include_prefix[:-8]
        elif include_prefix.endswith('__feedback'):
            include_prefix = include_prefix[:-10]
        includes.setdefault(include_prefix + '__struct.h', []).append(member.name)
        includes.setdefault(include_prefix + '__functions.h', []).append(member.name)
}@
@[if includes]@
// Include directives for member types
@[    for header_file, member_names in includes.items()]@
@[        for member_name in member_names]@
// Member '@(member_name)'
@[        end for]@
@[        if header_file in include_directives]@
// already included above
// @
@[        else]@
@{include_directives.add(header_file)}@
@[        end if]@
#include "@(header_file)"
@[    end for]@
@[end if]@

// forward declare type support functions
@{
from collections import OrderedDict
forward_declares = OrderedDict()
for member in message.structure.members:
    type_ = member.type
    if isinstance(type_, AbstractNestedType):
       type_ = type_.value_type
    if isinstance(type_, NamespacedType):
        _, member_names = forward_declares.setdefault(type_.name, (type_, []))
        member_names.append(member.name)
}@
@[for member_type, member_names in forward_declares.values()]@
@[  for name in member_names]@
// Member '@(name)'
@[  end for]@
@[  if member_type.namespaces[0] != package_name]@
ROSIDL_TYPESUPPORT_CONNEXT_C_IMPORT_@(package_name)
@[  end if]@
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_c,
  @(', '.join(member_type.namespaces)),
  @(member_type.name))();
@[end for]@

@# // Make callback functions specific to this message type.
@{
__ros_c_msg_type = '__'.join(message.structure.namespaced_type.namespaced_name())
__dds_cpp_msg_type_prefix = '::'.join(message.structure.namespaced_type.namespaces + ['dds_', message.structure.namespaced_type.name])
__dds_cpp_msg_type = __dds_cpp_msg_type_prefix + '_'
}@
static DDS_TypeCode *
_@(message.structure.namespaced_type.name)__get_type_code()
{
  return @(__dds_cpp_msg_type_prefix)_TypeSupport::get_typecode();
}

static bool
_@(message.structure.namespaced_type.name)__convert_ros_to_dds(const void * untyped_ros_message, void * untyped_dds_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  if (!untyped_dds_message) {
    fprintf(stderr, "dds message handle is null\n");
    return false;
  }
  const @(__ros_c_msg_type) * ros_message =
    static_cast<const @(__ros_c_msg_type) *>(untyped_ros_message);
  @(__dds_cpp_msg_type) * dds_message =
    static_cast<@(__dds_cpp_msg_type) *>(untyped_dds_message);
@[if not message.structure.members]@
  // No fields is a no-op.
  (void)dds_message;
  (void)ros_message;
@[end if]@
@[for member in message.structure.members]@
  // Member name: @(member.name)
  {
@{
type_ = member.type
if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type
}@
@[  if isinstance(type_, NamespacedType)]@
    const message_type_support_callbacks_t * @('__'.join(type_.namespaced_name()))__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_connext_c, @(', '.join(type_.namespaced_name()))
      )()->data);
@[  end if]@
@[  if isinstance(member.type, AbstractNestedType)]@
@[    if isinstance(member.type, Array)]@
    size_t size = @(member.type.size);
@[    else]@
    size_t size = ros_message->@(member.name).size;
    if (size > (std::numeric_limits<DDS_Long>::max)()) {
      fprintf(stderr, "array size exceeds maximum DDS sequence size\n");
      return false;
    }
@[      if isinstance(member.type, BoundedSequence)]@
    if (size > @(member.type.maximum_size)) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
@[      end if]@
    DDS_Long length = static_cast<DDS_Long>(size);
    if (length > dds_message->@(member.name)_.maximum()) {
      if (!dds_message->@(member.name)_.maximum(length)) {
        fprintf(stderr, "failed to set maximum of sequence\n");
        return false;
      }
    }
    if (!dds_message->@(member.name)_.length(length)) {
      fprintf(stderr, "failed to set length of sequence\n");
      return false;
    }
@[    end if]@
    for (DDS_Long i = 0; i < static_cast<DDS_Long>(size); ++i) {
@[    if isinstance(member.type, Array)]@
      auto & ros_i = ros_message->@(member.name)[i];
@[    else]@
      auto & ros_i = ros_message->@(member.name).data[i];
@[    end if]@
@[    if isinstance(type_, AbstractString)]@
      const rosidl_generator_c__String * str = &ros_i;
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      dds_message->@(member.name)_[static_cast<DDS_Long>(i)] = DDS_String_dup(str->data);
@[    elif isinstance(type_, AbstractWString)]@
      const rosidl_generator_c__U16String * str = &ros_i;
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != u'\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      DDS_Wchar * wstr = rosidl_typesupport_connext_c::create_wstring_from_u16string(*str);
      if (NULL == wstr) {
        fprintf(stderr, "failed to create wstring from u16string\n");
        return false;
      }
      dds_message->@(member.name)_[static_cast<DDS_Long>(i)] = wstr;
@[    elif isinstance(type_, BasicType)]@
@[      if type_.typename == 'boolean']@
      dds_message->@(member.name)_[i] = 1 ? ros_i : 0;
@[      else]@
      dds_message->@(member.name)_[i] = ros_i;
@[      end if]@
@[    else]@
      if (!@(idl_structure_type_to_c_typename(type_))__callbacks->convert_ros_to_dds(
          &ros_i, &dds_message->@(member.name)_[i]))
      {
        return false;
      }
@[    end if]@
    }
@[  elif isinstance(member.type, AbstractString)]@
    const rosidl_generator_c__String * str = &ros_message->@(member.name);
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    dds_message->@(member.name)_ = DDS_String_dup(str->data);
@[  elif isinstance(member.type, AbstractWString)]@
    const rosidl_generator_c__U16String * str = &ros_message->@(member.name);
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != u'\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    DDS_Wchar * wstr = rosidl_typesupport_connext_c::create_wstring_from_u16string(*str);
    if (NULL == wstr) {
      fprintf(stderr, "failed to create wstring from u16string\n");
      return false;
    }
    dds_message->@(member.name)_ = wstr;
@[  elif isinstance(member.type, BasicType)]@
    dds_message->@(member.name)_ = ros_message->@(member.name);
@[  else]@
    if (!@(idl_structure_type_to_c_typename(member.type))__callbacks->convert_ros_to_dds(
        &ros_message->@(member.name), &dds_message->@(member.name)_))
    {
      return false;
    }
@[  end if]@
  }
@[end for]@
  return true;
}

static bool
_@(message.structure.namespaced_type.name)__convert_dds_to_ros(const void * untyped_dds_message, void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  if (!untyped_dds_message) {
    fprintf(stderr, "dds message handle is null\n");
    return false;
  }
  const @(__dds_cpp_msg_type) * dds_message =
    static_cast<const @(__dds_cpp_msg_type) *>(untyped_dds_message);
  @(__ros_c_msg_type) * ros_message =
    static_cast<@(__ros_c_msg_type) *>(untyped_ros_message);
@[if not message.structure.members]@
  // No fields is a no-op.
  (void)dds_message;
  (void)ros_message;
@[end if]@
@[for member in message.structure.members]@
  // Member name: @(member.name)
  {
@{
type_ = member.type
if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type
}@
@[  if isinstance(member.type, AbstractNestedType)]@
@[    if isinstance(member.type, Array)]@
    DDS_Long size = @(member.type.size);
@[    else]@
    DDS_Long size = dds_message->@(member.name)_.length();
    if (ros_message->@(member.name).data) {
      @(idl_type_to_c(member.type) + '__fini')(&ros_message->@(member.name));
    }
    if (!@(idl_type_to_c(member.type) + '__init')(&ros_message->@(member.name), size)) {
      return "failed to create array for field '@(member.name)'";
    }
@[    end if]@
    for (DDS_Long i = 0; i < size; i++) {
@[    if isinstance(member.type, Array)]@
      auto & ros_i = ros_message->@(member.name)[i];
@[    else]@
      auto & ros_i = ros_message->@(member.name).data[i];
@[    end if]@
@[    if isinstance(type_, BasicType)]@
@[      if type_.typename == 'boolean']@
      ros_i = (dds_message->@(member.name)_[i] != 0);
@[      else]@
      ros_i = dds_message->@(member.name)_[i];
@[      end if]@
@[    elif isinstance(type_, AbstractString)]@
      if (!ros_i.data) {
        rosidl_generator_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_generator_c__String__assign(
        &ros_i,
        dds_message->@(member.name)_[i]);
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field '@(member.name)'\n");
        return false;
      }
@[    elif isinstance(type_, AbstractWString)]@
      if (!ros_i.data) {
        rosidl_generator_c__U16String__init(&ros_i);
      }
      bool succeeded = rosidl_typesupport_connext_c::wstring_to_u16string(dds_message->@(member.name)_[i], ros_i);
      if (!succeeded) {
        fprintf(stderr, "failed to create wstring from u16string\n");
        rosidl_generator_c__U16String__fini(&ros_i);
        return false;
      }
@[    elif isinstance(type_, NamespacedType)]@
      const rosidl_message_type_support_t * ts =
        ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_connext_c,
        @(', '.join(type_.namespaces)),
        @(type_.name))();
      const message_type_support_callbacks_t * callbacks =
        static_cast<const message_type_support_callbacks_t *>(ts->data);
      callbacks->convert_dds_to_ros(&dds_message->@(member.name)_[i], &ros_i);
@[    else]@
@{      assert False, 'Unknown member base type'}@
@[    end if]@
    }
@[  elif isinstance(member.type, AbstractString)]@
    if (!ros_message->@(member.name).data) {
      rosidl_generator_c__String__init(&ros_message->@(member.name));
    }
    bool succeeded = rosidl_generator_c__String__assign(
      &ros_message->@(member.name),
      dds_message->@(member.name)_);
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field '@(member.name)'\n");
      return false;
    }
@[  elif isinstance(member.type, AbstractWString)]@
    if (!ros_message->@(member.name).data) {
      rosidl_generator_c__U16String__init(&ros_message->@(member.name));
    }
    bool succeeded = rosidl_typesupport_connext_c::wstring_to_u16string(dds_message->@(member.name)_, ros_message->@(member.name));
    if (!succeeded) {
      fprintf(stderr, "failed to create wstring from u16string\n");
      rosidl_generator_c__U16String__fini(&ros_message->@(member.name));
      return false;
    }
@[  elif isinstance(member.type, BasicType)]@
    ros_message->@(member.name) = dds_message->@(member.name)_@(' == static_cast<DDS_Boolean>(true)' if member.type.typename == 'boolean' else '');
@[  elif isinstance(member.type, NamespacedType)]@
    const rosidl_message_type_support_t * ts =
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
      rosidl_typesupport_connext_c,
      @(', '.join(member.type.namespaces)),
      @(member.type.name))();
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(ts->data);
    callbacks->convert_dds_to_ros(&dds_message->@(member.name)_, &ros_message->@(member.name));
@[  else]@
@{    assert False, 'Unknown member type'}@
@[  end if]@
  }
@[end for]@
  return true;
}


static bool
_@(message.structure.namespaced_type.name)__to_cdr_stream(
  const void * untyped_ros_message,
  rcutils_uint8_array_t * cdr_stream)
{
  if (!untyped_ros_message) {
    return false;
  }
  if (!cdr_stream) {
    return false;
  }
  const @(__ros_c_msg_type) * ros_message =
    static_cast<const @(__ros_c_msg_type) *>(untyped_ros_message);
  @(__dds_cpp_msg_type) dds_message;
  if (!_@(message.structure.namespaced_type.name)__convert_ros_to_dds(ros_message, &dds_message)) {
    return false;
  }

  // call the serialize function for the first time to get the expected length of the message
  unsigned int expected_length;
  if (@(__dds_cpp_msg_type_prefix)_Plugin_serialize_to_cdr_buffer(
      NULL, &expected_length, &dds_message) != RTI_TRUE)
  {
    fprintf(stderr, "failed to call @(__dds_cpp_msg_type_prefix)_Plugin_serialize_to_cdr_buffer()\n");
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
  if (@(__dds_cpp_msg_type_prefix)_Plugin_serialize_to_cdr_buffer(
      reinterpret_cast<char *>(cdr_stream->buffer),
      &buffer_length_uint,
      &dds_message) != RTI_TRUE)
  {
    return false;
  }

  return true;
}

static bool
_@(message.structure.namespaced_type.name)__to_message(
  const rcutils_uint8_array_t * cdr_stream,
  void * untyped_ros_message)
{
  if (!cdr_stream) {
    return false;
  }
  if (!untyped_ros_message) {
    return false;
  }

  @(__dds_cpp_msg_type) * dds_message =
    @(__dds_cpp_msg_type_prefix)_TypeSupport::create_data();
  if (cdr_stream->buffer_length > (std::numeric_limits<unsigned int>::max)()) {
    fprintf(stderr, "cdr_stream->buffer_length, unexpectedly larger than max unsigned int\n");
    return false;
  }
  if (@(__dds_cpp_msg_type_prefix)_Plugin_deserialize_from_cdr_buffer(
      dds_message,
      reinterpret_cast<char *>(cdr_stream->buffer),
      static_cast<unsigned int>(cdr_stream->buffer_length)) != RTI_TRUE)
  {
    fprintf(stderr, "deserialize from cdr buffer failed\n");
    return false;
  }
  bool success = _@(message.structure.namespaced_type.name)__convert_dds_to_ros(dds_message, untyped_ros_message);
  if (@(__dds_cpp_msg_type_prefix)_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return success;
}

@# // Collect the callback functions and provide a function to get the type support struct.
static message_type_support_callbacks_t _@(message.structure.namespaced_type.name)__callbacks = {
  "@('::'.join([package_name] + list(interface_path.parents[0].parts)))",  // message_namespace
  "@(message.structure.namespaced_type.name)",  // message_name
  _@(message.structure.namespaced_type.name)__get_type_code,  // get_type_code
  _@(message.structure.namespaced_type.name)__convert_ros_to_dds,  // convert_ros_to_dds
  _@(message.structure.namespaced_type.name)__convert_dds_to_ros,  // convert_dds_to_ros
  _@(message.structure.namespaced_type.name)__to_cdr_stream,  // to_cdr_stream
  _@(message.structure.namespaced_type.name)__to_message  // to_message
};

static rosidl_message_type_support_t _@(message.structure.namespaced_type.name)__type_support = {
  rosidl_typesupport_connext_c__identifier,
  &_@(message.structure.namespaced_type.name)__callbacks,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_c,
  @(', '.join([package_name] + list(interface_path.parents[0].parts))),
  @(message.structure.namespaced_type.name))()
{
  return &_@(message.structure.namespaced_type.name)__type_support;
}

#if defined(__cplusplus)
}
#endif
