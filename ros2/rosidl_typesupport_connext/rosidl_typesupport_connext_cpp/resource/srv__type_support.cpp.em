@# Included from rosidl_typesupport_connext_cpp/resource/idl__dds_connext__type_support.cpp.em
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
include_parts = [package_name] + list(interface_path.parents[0].parts)
include_base = '/'.join(include_parts)
cpp_include_prefix = interface_path.stem

c_include_prefix = convert_camel_case_to_lower_case_underscore(cpp_include_prefix)

header_files = [
    include_base + '/' + c_include_prefix + '__rosidl_typesupport_connext_cpp.hpp',
    'rmw/error_handling.h',
    'rosidl_typesupport_connext_cpp/identifier.hpp',
    'rosidl_typesupport_connext_cpp/service_type_support.h',
    'rosidl_typesupport_connext_cpp/service_type_support_decl.hpp',
    include_base + '/' + c_include_prefix + '__struct.hpp',
    include_base + '/dds_connext/' + cpp_include_prefix + '_Support.h',
    include_base + '/dds_connext/' + cpp_include_prefix + '_Plugin.h'
]

dds_specific_header_files = [
    'ndds/connext_cpp/connext_cpp_requester_details.h',
    'ndds/ndds_cpp.h',
    'ndds/ndds_requestreply_cpp.h'
]
}@
#ifdef Connext_GLIBCXX_USE_CXX11_ABI_ZERO
#define _GLIBCXX_USE_CXX11_ABI 0
#endif

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
#include <@(header_file)>
@[end for]@

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

@[for header_file in header_files]@
@[    if header_file in include_directives]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
#include "@(header_file)"
@[end for]@

@{
TEMPLATE(
    'msg__type_support.cpp.em',
    package_name=package_name, interface_path=interface_path,
    message=service.request_message,
    include_directives=include_directives
)
}@

@{
TEMPLATE(
    'msg__type_support.cpp.em',
    package_name=package_name, interface_path=interface_path,
    message=service.response_message,
    include_directives=include_directives
)
}@

class DDSDomainParticipant;
class DDSDataReader;
struct DDS_SampleIdentity_t;

@[for ns in service.namespaced_type.namespaces]@

namespace @(ns)
{
@[end for]@
namespace typesupport_connext_cpp
{
@{
__ros_srv_pkg_prefix = '::'.join(service.namespaced_type.namespaces)
__ros_srv_type = __ros_srv_pkg_prefix + '::' + service.namespaced_type.name
__ros_request_msg_type = __ros_srv_pkg_prefix + '::' + service.request_message.structure.namespaced_type.name
__ros_response_msg_type = __ros_srv_pkg_prefix + '::' + service.response_message.structure.namespaced_type.name
__dds_request_msg_type = __ros_srv_pkg_prefix + '::dds_::' + service.request_message.structure.namespaced_type.name + '_'
__dds_response_msg_type = __ros_srv_pkg_prefix + '::dds_::' + service.response_message.structure.namespaced_type.name + '_'
}@

void * create_requester__@(service.namespaced_type.name)(
  void * untyped_participant,
  const char * request_topic_str,
  const char * response_topic_str,
  const void * untyped_datareader_qos,
  const void * untyped_datawriter_qos,
  void ** untyped_reader,
  void ** untyped_writer,
  void * (*allocator)(size_t))
{
  using RequesterType = connext::Requester<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  if (!untyped_participant || !request_topic_str || !response_topic_str || !untyped_reader) {
    return NULL;
  }
  auto _allocator = allocator ? allocator : &malloc;

  DDSDomainParticipant * participant = static_cast<DDSDomainParticipant *>(untyped_participant);
  const DDS_DataReaderQos * datareader_qos = static_cast<const DDS_DataReaderQos *>(untyped_datareader_qos);
  const DDS_DataWriterQos * datawriter_qos = static_cast<const DDS_DataWriterQos *>(untyped_datawriter_qos);
  connext::RequesterParams requester_params(participant);

  // we create separate publishers and subscribers
  // because the default publisher is a singleton within
  // the replier/requester context in connext.
  // if we use the implicit one, every service/client will
  // overwrite the QoS of all previous instances.
  DDS::Publisher * publisher = participant->create_publisher(
    DDS_PUBLISHER_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
  if (publisher == NULL) {
    RMW_SET_ERROR_MSG("C++ exception during construction of publisher for requester");
    return NULL;
  }
  DDS::Subscriber * subscriber = participant->create_subscriber(
    DDS_SUBSCRIBER_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
  if (subscriber == NULL) {
    RMW_SET_ERROR_MSG("C++ exception during construction of subscriber for requester");
    return NULL;
  }
  requester_params.publisher(publisher);
  requester_params.subscriber(subscriber);
  requester_params.request_topic_name(request_topic_str);
  requester_params.reply_topic_name(response_topic_str);
  requester_params.datareader_qos(*datareader_qos);
  requester_params.datawriter_qos(*datawriter_qos);

  RequesterType * requester = static_cast<RequesterType *>(_allocator(sizeof(RequesterType)));
  try {
    new (requester) RequesterType(requester_params);
  } catch (...) {
    RMW_SET_ERROR_MSG("C++ exception during construction of Requester");
    return NULL;
  }

  *untyped_reader = requester->get_reply_datareader();
  *untyped_writer = requester->get_request_datawriter();
  return requester;
}

const char * destroy_requester__@(service.namespaced_type.name)(
  void * untyped_requester,
  void (* deallocator)(void *))
{
  using RequesterType = connext::Requester<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  auto requester = static_cast<RequesterType *>(untyped_requester);

  requester->~RequesterType();
  auto _deallocator = deallocator ? deallocator : &free;
  _deallocator(requester);
  return nullptr;
}

int64_t send_request__@(service.namespaced_type.name)(
  void * untyped_requester,
  const void * untyped_ros_request)
{
  using ROSRequestType = @(__ros_request_msg_type);
  using RequesterType = connext::Requester<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  connext::WriteSample<
    @(__dds_request_msg_type)> request;
  const ROSRequestType & ros_request = *(
    static_cast<const ROSRequestType *>(untyped_ros_request));
  @(__ros_srv_pkg_prefix)::typesupport_connext_cpp::convert_ros_message_to_dds(
    ros_request, request.data());

  RequesterType * requester = static_cast<RequesterType *>(untyped_requester);

  requester->send_request(request);
  int64_t sequence_number = ((int64_t)request.identity().sequence_number.high) << 32 |
    request.identity().sequence_number.low;
  return sequence_number;
}

void * create_replier__@(service.namespaced_type.name)(
  void * untyped_participant,
  const char * request_topic_str,
  const char * response_topic_str,
  const void * untyped_datareader_qos,
  const void * untyped_datawriter_qos,
  void ** untyped_reader,
  void ** untyped_writer,
  void * (*allocator)(size_t))
{
  using ReplierType = connext::Replier<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  if (!untyped_participant || !request_topic_str || !response_topic_str || !untyped_reader) {
    return NULL;
  }
  auto _allocator = allocator ? allocator : &malloc;

  DDSDomainParticipant * participant = static_cast<DDSDomainParticipant *>(untyped_participant);
  const DDS_DataReaderQos * datareader_qos = static_cast<const DDS_DataReaderQos *>(untyped_datareader_qos);
  const DDS_DataWriterQos * datawriter_qos = static_cast<const DDS_DataWriterQos *>(untyped_datawriter_qos);
  connext::ReplierParams<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  > replier_params(participant);

  // we create separate publishers and subscribers
  // because the default publisher is a singleton within
  // the replier/requester context in connext.
  // if we use the implicit one, every service/client will
  // overwrite the QoS of all previous instances.
  DDS::Publisher * publisher = participant->create_publisher(
    DDS_PUBLISHER_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
  if (publisher == NULL) {
    RMW_SET_ERROR_MSG("C++ exception during construction of publisher for replier");
    return NULL;
  }
  DDS::Subscriber * subscriber = participant->create_subscriber(
    DDS_SUBSCRIBER_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
  if (subscriber == NULL) {
    RMW_SET_ERROR_MSG("C++ exception during construction of subscriber for replier");
    return NULL;
  }
  replier_params.publisher(publisher);
  replier_params.subscriber(subscriber);
  replier_params.request_topic_name(request_topic_str);
  replier_params.reply_topic_name(response_topic_str);
  replier_params.datareader_qos(*datareader_qos);
  replier_params.datawriter_qos(*datawriter_qos);

  ReplierType * replier = static_cast<ReplierType *>(_allocator(sizeof(ReplierType)));
  try {
    new (replier) ReplierType(replier_params);
  } catch (...) {
    RMW_SET_ERROR_MSG("C++ exception during construction of Requester");
    return NULL;
  }

  *untyped_reader = replier->get_request_datareader();
  *untyped_writer = replier->get_reply_datawriter();
  return replier;
}

const char * destroy_replier__@(service.namespaced_type.name)(
  void * untyped_replier,
  void (* deallocator)(void *))
{
  using ReplierType = connext::Replier<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  auto replier = static_cast<ReplierType *>(untyped_replier);

  replier->~ReplierType();
  auto _deallocator = deallocator ? deallocator : &free;
  _deallocator(replier);
  return nullptr;
}

bool take_request__@(service.namespaced_type.name)(
  void * untyped_replier,
  rmw_request_id_t * request_header,
  void * untyped_ros_request)
{
  using ReplierType = connext::Replier<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  using ROSRequestType = @(__ros_request_msg_type);
  if (!untyped_replier || !request_header || !untyped_ros_request) {
    return false;
  }

  ReplierType * replier = static_cast<ReplierType *>(untyped_replier);

  ROSRequestType & ros_request = *static_cast<ROSRequestType *>(untyped_ros_request);

  connext::Sample<@(__dds_request_msg_type)> request;
  bool taken = replier->take_request(request);
  if (!taken) {
    return false;
  }
  if (!request.info().valid_data) {
    return false;
  }

  bool converted =
    @(__ros_srv_pkg_prefix)::typesupport_connext_cpp::convert_dds_message_to_ros(request.data(), ros_request);
  if (!converted) {
    return false;
  }

  size_t SAMPLE_IDENTITY_SIZE = 16;
  memcpy(&(request_header->writer_guid[0]), request.identity().writer_guid.value, SAMPLE_IDENTITY_SIZE);

  request_header->sequence_number = ((int64_t)request.identity().sequence_number.high) << 32 | request.identity().sequence_number.low;
  return true;
}

bool take_response__@(service.namespaced_type.name)(
  void * untyped_requester,
  rmw_request_id_t * request_header,
  void * untyped_ros_response)
{
  using RequesterType = connext::Requester<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  using ROSResponseType = @(__ros_response_msg_type);
  if (!untyped_requester || !request_header || !untyped_ros_response) {
    return false;
  }

  RequesterType * requester = static_cast<RequesterType *>(untyped_requester);

  ROSResponseType & ros_response = *static_cast<ROSResponseType *>(untyped_ros_response);

  connext::Sample<@(__dds_response_msg_type)> response;
  bool received = requester->take_reply(response);
  if (!received) {
    return false;
  }
  if (!response.info().valid_data) {
    return false;
  }

  int64_t sequence_number =
    (((int64_t)response.related_identity().sequence_number.high) << 32) |
    response.related_identity().sequence_number.low;
  request_header->sequence_number = sequence_number;

  bool converted =
    @(__ros_srv_pkg_prefix)::typesupport_connext_cpp::convert_dds_message_to_ros(response.data(), ros_response);
  return converted;
}

bool send_response__@(service.namespaced_type.name)(
  void * untyped_replier,
  const rmw_request_id_t * request_header,
  const void * untyped_ros_response)
{
  using ROSResponseType = const @(__ros_response_msg_type);
  using ReplierType = connext::Replier<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  if (!untyped_replier || !request_header || !untyped_ros_response) {
    return false;
  }

  connext::WriteSample<@(__dds_response_msg_type)> response;
  ROSResponseType & ros_response = *(reinterpret_cast<ROSResponseType *>(untyped_ros_response));
  bool converted =
    @(__ros_srv_pkg_prefix)::typesupport_connext_cpp::convert_ros_message_to_dds(ros_response, response.data());
  if (!converted) {
    return false;
  }

  DDS_SampleIdentity_t request_identity;

  size_t SAMPLE_IDENTITY_SIZE = 16;
  memcpy(request_identity.writer_guid.value, &request_header->writer_guid[0], SAMPLE_IDENTITY_SIZE);

  request_identity.sequence_number.high = (int32_t)((request_header->sequence_number & 0xFFFFFFFF00000000) >> 32);
  request_identity.sequence_number.low = (uint32_t)(request_header->sequence_number & 0xFFFFFFFF);

  ReplierType * replier = static_cast<ReplierType *>(untyped_replier);

  replier->send_reply(response, request_identity);
  return true;
}

void *
get_request_datawriter__@(service.namespaced_type.name)(void * untyped_requester)
{
  if (!untyped_requester) {
    return NULL;
  }
  using RequesterType = connext::Requester<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  RequesterType * requester = reinterpret_cast<RequesterType *>(untyped_requester);
  return requester->get_request_datawriter();
}

void *
get_reply_datareader__@(service.namespaced_type.name)(void * untyped_requester)
{
  if (!untyped_requester) {
    return NULL;
  }
  using RequesterType = connext::Requester<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  RequesterType * requester = reinterpret_cast<RequesterType *>(untyped_requester);
  return requester->get_reply_datareader();
}

void *
get_request_datareader__@(service.namespaced_type.name)(void * untyped_replier)
{
  if (!untyped_replier) {
    return NULL;
  }
  using ReplierType = connext::Replier<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  ReplierType * replier = reinterpret_cast<ReplierType *>(untyped_replier);
  return replier->get_request_datareader();
}

void *
get_reply_datawriter__@(service.namespaced_type.name)(void * untyped_replier)
{
  if (!untyped_replier) {
    return NULL;
  }
  using ReplierType = connext::Replier<
    @(__dds_request_msg_type),
    @(__dds_response_msg_type)
  >;
  ReplierType * replier = reinterpret_cast<ReplierType *>(untyped_replier);
  return replier->get_reply_datawriter();
}

static service_type_support_callbacks_t _@(service.namespaced_type.name)__callbacks = {
  "@('::'.join([package_name] + list(interface_path.parents[0].parts)))",
  "@(service.namespaced_type.name)",
  &create_requester__@(service.namespaced_type.name),
  &destroy_requester__@(service.namespaced_type.name),
  &create_replier__@(service.namespaced_type.name),
  &destroy_replier__@(service.namespaced_type.name),
  &send_request__@(service.namespaced_type.name),
  &take_request__@(service.namespaced_type.name),
  &send_response__@(service.namespaced_type.name),
  &take_response__@(service.namespaced_type.name),
  &get_request_datawriter__@(service.namespaced_type.name),
  &get_reply_datareader__@(service.namespaced_type.name),
  &get_request_datareader__@(service.namespaced_type.name),
  &get_reply_datawriter__@(service.namespaced_type.name),
};

static rosidl_service_type_support_t _@(service.namespaced_type.name)__handle = {
  rosidl_typesupport_connext_cpp::typesupport_identifier,
  &_@(service.namespaced_type.name)__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_connext_cpp

@[for ns in reversed(service.namespaced_type.namespaces)]@
}  // namespace @(ns)

@[end for]@

namespace rosidl_typesupport_connext_cpp
{

template<>
ROSIDL_TYPESUPPORT_CONNEXT_CPP_EXPORT_@(package_name)
const rosidl_service_type_support_t *
get_service_type_support_handle<@(__ros_srv_type)>()
{
  return &@(__ros_srv_pkg_prefix)::typesupport_connext_cpp::_@(service.namespaced_type.name)__handle;
}

}  // namespace rosidl_typesupport_connext_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_connext_cpp,
  @(', '.join([package_name] + list(interface_path.parents[0].parts))),
  @(service.namespaced_type.name))()
{
  return &@(__ros_srv_pkg_prefix)::typesupport_connext_cpp::_@(service.namespaced_type.name)__handle;
}

#ifdef __cplusplus
}
#endif
