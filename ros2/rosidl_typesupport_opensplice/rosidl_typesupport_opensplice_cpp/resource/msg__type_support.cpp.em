// generated from rosidl_typesupport_opensplice_cpp/resource/msg__type_support.cpp.em
// generated code does not contain a copyright notice

@# Included from rosidl_typesupport_opensplice_cpp/resource/idl__dds_opensplice__type_support.cpp.em
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import NamespacedType
include_parts = [package_name] + list(interface_path.parents[0].parts)
include_dir = '/'.join(include_parts)
include_parts.append(convert_camel_case_to_lower_case_underscore(interface_path.stem))
include_base = '/'.join(include_parts)
header_file = include_base +'__rosidl_typesupport_opensplice_cpp.hpp'
}@
@{
header_files = [
  header_file,
  include_base + '__rosidl_typesupport_opensplice_cpp.hpp',
  'rosidl_typesupport_cpp/message_type_support.hpp',
  include_base + '__struct.hpp',
  include_dir + '/dds_opensplice/ccpp_' + interface_path.stem + '_.h',
  'rosidl_typesupport_opensplice_cpp/identifier.hpp',
  'rosidl_typesupport_opensplice_cpp/message_type_support.h',
  'rosidl_typesupport_opensplice_cpp/message_type_support_decl.hpp',
  'rosidl_typesupport_opensplice_cpp/u__instanceHandle.h',
  'rmw/rmw.h'
]
}@
@[for header_file in header_files]@
@[    if header_file in include_directives]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
@[    if '/' not in header_file]@
#include <@(header_file)>
@[    else]@
#include "@(header_file)"
@[    end if]@
@[end for]@

@{
__ros_msg_pkg_prefix = '::'.join(message.structure.namespaced_type.namespaces)
__ros_msg_type_prefix = __ros_msg_pkg_prefix + '::' + message.structure.namespaced_type.name
__dds_msg_type_prefix = __ros_msg_pkg_prefix + '::dds_::' +  message.structure.namespaced_type.name + '_'
}@

// forward declaration of message dependencies and their conversion functions
@[for member in message.structure.members]@
@{
_type = member.type
if isinstance(_type, AbstractNestedType):
   _type = member.type.value_type
}@
@[  if isinstance(_type, NamespacedType)]@
@[    for ns in _type.namespaces]@
namespace @(ns)
{
@[    end for]@
namespace dds_
{
struct @(_type.name)_;
}  // namespace dds_
namespace typesupport_opensplice_cpp
{
void convert_ros_message_to_dds(
  const @('::'.join(_type.namespaced_name())) &,
  @('::'.join(_type.namespaces))::dds_::@(_type.name)_ &);
void convert_dds_message_to_ros(
  const @('::'.join(_type.namespaces))::dds_::@(_type.name)_ &,
  @('::'.join(_type.namespaced_name())) &);
}  // namespace typesupport_opensplice_cpp
@[    for ns in reversed(_type.namespaces)]@
}  // namespace @(ns)
@[    end for]@
@[  end if]@
@[end for]@

@[for ns in message.structure.namespaced_type.namespaces]@
namespace @(ns)
{
@[end for]@

namespace typesupport_opensplice_cpp
{

using __dds_msg_type_@(message.structure.namespaced_type.name) = @(__dds_msg_type_prefix);
using __ros_msg_type_@(message.structure.namespaced_type.name) = @(__ros_msg_type_prefix);

static @(__dds_msg_type_prefix)TypeSupport __type_support_@(message.structure.namespaced_type.name);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_@(package_name)
const char *
register_type__@(message.structure.namespaced_type.name)(
  void * untyped_participant,
  const char * type_name)
{
  if (!untyped_participant) {
    return "untyped participant handle is null";
  }
  if (!type_name) {
    return "type name handle is null";
  }
  DDS::DomainParticipant * participant =
    static_cast<DDS::DomainParticipant *>(untyped_participant);

  DDS::ReturnCode_t status = __type_support_@(message.structure.namespaced_type.name).register_type(participant, type_name);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "@(__dds_msg_type_prefix)TypeSupport.register_type: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "@(__dds_msg_type_prefix)TypeSupport.register_type: "
             "bad domain participant or type name parameter";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "@(__dds_msg_type_prefix)TypeSupport.register_type: "
             "out of resources";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "@(__dds_msg_type_prefix)TypeSupport.register_type: "
             "already registered with a different TypeSupport class";
    case DDS::RETCODE_OK:
      return nullptr;
    default:
      return "@(__dds_msg_type_prefix)TypeSupport.register_type: unknown return code";
  }
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_@(package_name)
void
convert_ros_message_to_dds(
  const __ros_msg_type_@(message.structure.namespaced_type.name) & ros_message,
  __dds_msg_type_@(message.structure.namespaced_type.name) & dds_message)
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
    if (size > static_cast<size_t>((std::numeric_limits<DDS::Long>::max)())) {
      throw std::runtime_error("array size exceeds maximum DDS sequence size");
    }
    DDS::Long length = static_cast<DDS::Long>(size);
    dds_message.@(member.name)_.length(length);
@[    end if]@
    for (DDS::ULong i = 0; i < size; i++) {
@[    if isinstance(member.type.value_type, AbstractString)]@
      dds_message.@(member.name)_[i] = ros_message.@(member.name)[i].c_str();
@[    elif isinstance(member.type.value_type, AbstractWString)]@
#ifndef _WIN32
      std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> cv;
      dds_message.@(member.name)_[i] = cv.to_bytes(ros_message.@(member.name)[i].c_str()).c_str();
#else
      std::wstring_convert<std::codecvt_utf8_utf16<int16_t>, int16_t> cv;
      static_assert(
        sizeof(int16_t) == sizeof(std::u16string::value_type),
        "sizeof of std::u16string.value_type doesn't match int16_t");
      static_assert(
        alignof(int16_t) == alignof(std::u16string::value_type),
        "alignof of std::u16string.value_type doesn't match int16_t");
      dds_message.@(member.name)_[i] = cv.to_bytes(reinterpret_cast<const int16_t *>(ros_message.@(member.name)[i].c_str())).c_str();
#endif
@[    elif isinstance(member.type.value_type, BasicType)]@
@[      if member.type.value_type.typename == 'boolean']@
      dds_message.@(member.name)_[i] = 1 ? ros_message.@(member.name)[i] : 0;
@[      else]@
      dds_message.@(member.name)_[i] = ros_message.@(member.name)[i];
@[      end if]@
@[    else]@
      @('::'.join(member.type.value_type.namespaces))::typesupport_opensplice_cpp::convert_ros_message_to_dds(
        ros_message.@(member.name)[i], dds_message.@(member.name)_[i]);
@[    end if]@
    }
  }
@[  elif isinstance(member.type, AbstractString)]@
  dds_message.@(member.name)_ = ros_message.@(member.name).c_str();
@[  elif isinstance(member.type, AbstractWString)]@
  {
#ifndef _WIN32
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> cv;
    dds_message.@(member.name)_ = cv.to_bytes(ros_message.@(member.name).c_str()).c_str();
#else
    std::wstring_convert<std::codecvt_utf8_utf16<int16_t>, int16_t> cv;
    static_assert(
      sizeof(int16_t) == sizeof(std::u16string::value_type),
      "sizeof of std::u16string.value_type doesn't match int16_t");
    static_assert(
      alignof(int16_t) == alignof(std::u16string::value_type),
      "alignof of std::u16string.value_type doesn't match int16_t");
    dds_message.@(member.name)_ = cv.to_bytes(reinterpret_cast<const int16_t *>(ros_message.@(member.name).c_str())).c_str();
#endif
  }
@[  elif isinstance(member.type, BasicType)]@
  dds_message.@(member.name)_ = ros_message.@(member.name);
@[  else]@
  @('::'.join(member.type.namespaces))::typesupport_opensplice_cpp::convert_ros_message_to_dds(
    ros_message.@(member.name), dds_message.@(member.name)_);
@[  end if]@
@[end for]@
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_@(package_name)
const char *
publish__@(message.structure.namespaced_type.name)(
  void * untyped_topic_writer,
  const void * untyped_ros_message)
{
  DDS::DataWriter * topic_writer = static_cast<DDS::DataWriter *>(untyped_topic_writer);

  const __ros_msg_type_@(message.structure.namespaced_type.name) & ros_message = *static_cast<const __ros_msg_type_@(message.structure.namespaced_type.name) *>(untyped_ros_message);
  __dds_msg_type_@(message.structure.namespaced_type.name) dds_message;
  convert_ros_message_to_dds(ros_message, dds_message);

  @(__dds_msg_type_prefix)DataWriter * data_writer =
    @(__dds_msg_type_prefix)DataWriter::_narrow(topic_writer);
  DDS::ReturnCode_t status = data_writer->write(dds_message, DDS::HANDLE_NIL);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "@(__dds_msg_type_prefix)DataWriter.write: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "@(__dds_msg_type_prefix)DataWriter.write: "
             "bad handle or instance_data parameter";
    case DDS::RETCODE_ALREADY_DELETED:
      return "@(__dds_msg_type_prefix)DataWriter.write: "
             "this @(__dds_msg_type_prefix)DataWriter has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "@(__dds_msg_type_prefix)DataWriter.write: "
             "out of resources";
    case DDS::RETCODE_NOT_ENABLED:
      return "@(__dds_msg_type_prefix)DataWriter.write: "
             "this @(__dds_msg_type_prefix)DataWriter is not enabled";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "@(__dds_msg_type_prefix)DataWriter.write: "
             "the handle has not been registered with this @(__dds_msg_type_prefix)DataWriter";
    case DDS::RETCODE_TIMEOUT:
      return "@(__dds_msg_type_prefix)DataWriter.write: "
             "writing resulted in blocking and then exceeded the timeout set by the "
             "max_blocking_time of the ReliabilityQosPolicy";
    case DDS::RETCODE_OK:
      return nullptr;
    default:
      return "@(__dds_msg_type_prefix)DataWriter.write: unknown return code";
  }
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_@(package_name)
void
convert_dds_message_to_ros(
  const __dds_msg_type_@(message.structure.namespaced_type.name) & dds_message,
  __ros_msg_type_@(message.structure.namespaced_type.name) & ros_message)
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
    for (DDS::ULong i = 0; i < size; i++) {
@[    if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'boolean']@
      ros_message.@(member.name)[i] = (dds_message.@(member.name)_[i] != 0);
@[    elif isinstance(member.type.value_type, (BasicType, AbstractString))]@
      ros_message.@(member.name)[i] = dds_message.@(member.name)_[i];
@[    elif isinstance(member.type.value_type, (BasicType, AbstractWString))]@
#ifndef _WIN32
      std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> cv;
      ros_message.@(member.name)[i] = cv.from_bytes(dds_message.@(member.name)_[i]);
#else
      std::wstring_convert<std::codecvt_utf8_utf16<int16_t>, int16_t> cv;
      ros_message.@(member.name)[i] = reinterpret_cast<const char16_t *>(cv.from_bytes(static_cast<const char *>(dds_message.@(member.name)_[i])).c_str());
#endif
@[    else]@
      @('::'.join(member.type.value_type.namespaces))::typesupport_opensplice_cpp::convert_dds_message_to_ros(
        dds_message.@(member.name)_[i], ros_message.@(member.name)[i]);
@[    end if]@
    }
  }
@[  elif isinstance(member.type, AbstractString)]@
  ros_message.@(member.name) = dds_message.@(member.name)_;
@[  elif isinstance(member.type, AbstractWString)]@
  {
#ifndef _WIN32
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> cv;
    ros_message.@(member.name) = cv.from_bytes(dds_message.@(member.name)_);
#else
    std::wstring_convert<std::codecvt_utf8_utf16<int16_t>, int16_t> cv;
    ros_message.@(member.name) = reinterpret_cast<const char16_t *>(cv.from_bytes(static_cast<const char *>(dds_message.@(member.name)_)).c_str());
#endif
  }
@[  elif isinstance(member.type, BasicType)]@
  ros_message.@(member.name) =
    @('(' if member.type.typename == 'boolean' else '')dds_message.@(member.name)_@(' != 0)' if member.type.typename == 'boolean' else '');
@[  else]@
  @('::'.join(member.type.namespaces))::typesupport_opensplice_cpp::convert_dds_message_to_ros(
    dds_message.@(member.name)_, ros_message.@(member.name));
@[  end if]@
@[end for]@
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_@(package_name)
const char *
take__@(message.structure.namespaced_type.name)(
  void * untyped_topic_reader,
  bool ignore_local_publications,
  void * untyped_ros_message,
  bool * taken,
  void * sending_publication_handle)
{
  if (untyped_ros_message == 0) {
    return "invalid ros message pointer";
  }

  DDS::DataReader * topic_reader = static_cast<DDS::DataReader *>(untyped_topic_reader);

  @(__dds_msg_type_prefix)DataReader * data_reader =
    @(__dds_msg_type_prefix)DataReader::_narrow(topic_reader);

  @(__dds_msg_type_prefix)Seq dds_messages;
  DDS::SampleInfoSeq sample_infos;
  DDS::ReturnCode_t status = data_reader->take(
    dds_messages,
    sample_infos,
    1,
    DDS::ANY_SAMPLE_STATE,
    DDS::ANY_VIEW_STATE,
    DDS::ANY_INSTANCE_STATE);

  const char * errs = nullptr;
  bool ignore_sample = false;

  switch (status) {
    case DDS::RETCODE_ERROR:
      errs = "@(__dds_msg_type_prefix)DataReader.take: "
        "an internal error has occurred";
      goto finally;
    case DDS::RETCODE_ALREADY_DELETED:
      errs = "@(__dds_msg_type_prefix)DataReader.take: "
        "this @(__dds_msg_type_prefix)DataReader has already been deleted";
      goto finally;
    case DDS::RETCODE_OUT_OF_RESOURCES:
      errs = "@(__dds_msg_type_prefix)DataReader.take: "
        "out of resources";
      goto finally;
    case DDS::RETCODE_NOT_ENABLED:
      errs = "@(__dds_msg_type_prefix)DataReader.take: "
        "this @(__dds_msg_type_prefix)DataReader is not enabled";
      goto finally;
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      errs = "@(__dds_msg_type_prefix)DataReader.take: "
        "a precondition is not met, one of: "
        "max_samples > maximum and max_samples != LENGTH_UNLIMITED, or "
        "the two sequences do not have matching parameters (length, maximum, release), or "
        "maximum > 0 and release is false.";
      goto finally;
    case DDS::RETCODE_NO_DATA:
      *taken = false;
      errs = nullptr;
      goto finally;
    case DDS::RETCODE_OK:
      break;
    default:
      errs = "@(__dds_msg_type_prefix)DataReader.take: unknown return code";
      goto finally;
  }

  {
    DDS::SampleInfo & sample_info = sample_infos[0];
    if (!sample_info.valid_data) {
      // skip sample without data
      ignore_sample = true;
    } else {
      DDS::InstanceHandle_t sender_handle = sample_info.publication_handle;
      auto sender_gid = u_instanceHandleToGID(sender_handle);
      if (ignore_local_publications) {
        // compare the system id from the sender and this receiver
        // if they are equal the sample has been sent from this process and should be ignored
        DDS::InstanceHandle_t receiver_handle = topic_reader->get_instance_handle();
        auto receiver_gid = u_instanceHandleToGID(receiver_handle);
        ignore_sample = sender_gid.systemId == receiver_gid.systemId;
      }
      // This is nullptr when being used with plain rmw_take, so check first.
      if (sending_publication_handle) {
        *static_cast<DDS::InstanceHandle_t *>(sending_publication_handle) = sender_handle;
      }
    }
  }

  if (!ignore_sample) {
    __ros_msg_type_@(message.structure.namespaced_type.name) & ros_message = *static_cast<__ros_msg_type_@(message.structure.namespaced_type.name) *>(untyped_ros_message);
    convert_dds_message_to_ros(dds_messages[0], ros_message);
    *taken = true;
  } else {
    *taken = false;
  }

finally:
  // Ensure the loan is returned.
  status = data_reader->return_loan(dds_messages, sample_infos);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "@(__dds_msg_type_prefix)DataReader.return_loan: "
             "an internal error has occurred";
    case DDS::RETCODE_ALREADY_DELETED:
      return "@(__dds_msg_type_prefix)DataReader.return_loan: "
             "this @(__dds_msg_type_prefix)DataReader has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "@(__dds_msg_type_prefix)DataReader.return_loan: "
             "out of resources";
    case DDS::RETCODE_NOT_ENABLED:
      return "@(__dds_msg_type_prefix)DataReader.return_loan: "
             "this @(__dds_msg_type_prefix)DataReader is not enabled";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "@(__dds_msg_type_prefix)DataReader.return_loan: "
             "a precondition is not met, one of: "
             "the data_values and info_seq do not belong to a single related pair, or "
             "the data_values and info_seq were not obtained from this "
             "@(__dds_msg_type_prefix)DataReader";
    case DDS::RETCODE_OK:
      break;
    default:
      return "@(__dds_msg_type_prefix)DataReader.return_loan failed with "
             "unknown return code";
  }

  return errs;
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_@(package_name)
const char *
serialize__@(message.structure.namespaced_type.name)(
  const void * untyped_ros_message,
  void * untyped_serialized_data)
{
  const __ros_msg_type_@(message.structure.namespaced_type.name) & ros_message = *static_cast<const __ros_msg_type_@(message.structure.namespaced_type.name) *>(untyped_ros_message);
  __dds_msg_type_@(message.structure.namespaced_type.name) dds_message;

  convert_ros_message_to_dds(ros_message, dds_message);

  DDS::OpenSplice::CdrTypeSupport cdr_ts(__type_support_@(message.structure.namespaced_type.name));
  DDS::OpenSplice::CdrSerializedData * serdata = nullptr;

  DDS::ReturnCode_t status = cdr_ts.serialize(&dds_message, &serdata);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "@(__dds_msg_type_prefix)TypeSupport.serialize: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "@(__dds_msg_type_prefix)TypeSupport.serialize: "
             "bad parameter";
    case DDS::RETCODE_ALREADY_DELETED:
      return "@(__dds_msg_type_prefix)TypeSupport.serialize: "
             "this @(__dds_msg_type_prefix)TypeSupport has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "@(__dds_msg_type_prefix)TypeSupport.serialize: "
             "out of resources";
    case DDS::RETCODE_OK:
      break;
    default:
      return "@(__dds_msg_type_prefix)TypeSupport.serialize failed with "
             "unknown return code";
  }

  rmw_serialized_message_t * serialized_data =
    static_cast<rmw_serialized_message_t *>(untyped_serialized_data);

  auto data_length = serdata->get_size();

  if (serialized_data->buffer_capacity < data_length) {
    if (rmw_serialized_message_resize(serialized_data, data_length) == RMW_RET_OK) {
      serialized_data->buffer_capacity = data_length;
    } else {
      delete serdata;
      return "@(__dds_msg_type_prefix)TypeSupport.serialize: "
             "unable to dynamically resize serialized message";
    }
  }

  serialized_data->buffer_length = data_length;
  serdata->get_data(serialized_data->buffer);

  delete serdata;

  return nullptr;
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_@(package_name)
const char *
deserialize__@(message.structure.namespaced_type.name)(
  const uint8_t * buffer,
  unsigned length,
  void * untyped_ros_message)
{
  __dds_msg_type_@(message.structure.namespaced_type.name) dds_message;

  DDS::OpenSplice::CdrTypeSupport cdr_ts(__type_support_@(message.structure.namespaced_type.name));

  DDS::ReturnCode_t status = cdr_ts.deserialize(buffer, length, &dds_message);

  switch (status) {
    case DDS::RETCODE_ERROR:
      return "@(__dds_msg_type_prefix)TypeSupport.deserialize: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "@(__dds_msg_type_prefix)TypeSupport.deserialize: "
             "bad parameter";
    case DDS::RETCODE_ALREADY_DELETED:
      return "@(__dds_msg_type_prefix)TypeSupport.deserialize: "
             "this @(__dds_msg_type_prefix)TypeSupport has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "@(__dds_msg_type_prefix)TypeSupport.deserialize: "
             "out of resources";
    case DDS::RETCODE_OK:
      break;
    default:
      return "@(__dds_msg_type_prefix)TypeSupport.deserialize failed with "
             "unknown return code";
  }

  __ros_msg_type_@(message.structure.namespaced_type.name) & ros_message = *static_cast<__ros_msg_type_@(message.structure.namespaced_type.name) *>(untyped_ros_message);
  convert_dds_message_to_ros(dds_message, ros_message);

  return nullptr;
}

static message_type_support_callbacks_t @(message.structure.namespaced_type.name)_callbacks = {
  "@('::'.join([package_name] + list(interface_path.parents[0].parts)))",
  "@(message.structure.namespaced_type.name)",
  &register_type__@(message.structure.namespaced_type.name),
  &publish__@(message.structure.namespaced_type.name),
  &take__@(message.structure.namespaced_type.name),
  &serialize__@(message.structure.namespaced_type.name),
  &deserialize__@(message.structure.namespaced_type.name),
  nullptr,  // convert ros to dds (handled differently for C++)
  nullptr,  // convert dds to ros (handled differently for C++)
};

static rosidl_message_type_support_t @(message.structure.namespaced_type.name)_handle = {
  rosidl_typesupport_opensplice_cpp::typesupport_identifier,
  &@(message.structure.namespaced_type.name)_callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_opensplice_cpp

@[for ns in reversed(message.structure.namespaced_type.namespaces)]@
}  // namespace @(ns)
@[end for]@

namespace rosidl_typesupport_opensplice_cpp
{

template<>
ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_@(package_name)
const rosidl_message_type_support_t *
get_message_type_support_handle<@(__ros_msg_type_prefix)>()
{
  return &@(__ros_msg_pkg_prefix)::typesupport_opensplice_cpp::@(message.structure.namespaced_type.name)_handle;
}

}  // namespace rosidl_typesupport_opensplice_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_opensplice_cpp,
  @(', '.join([package_name] + list(interface_path.parents[0].parts))),
  @(message.structure.namespaced_type.name))()
{
  return &@(__ros_msg_pkg_prefix)::typesupport_opensplice_cpp::@(message.structure.namespaced_type.name)_handle;
}

#ifdef __cplusplus
}
#endif
