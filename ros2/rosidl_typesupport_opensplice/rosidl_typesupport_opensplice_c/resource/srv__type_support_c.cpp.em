// generated from rosidl_typesupport_opensplice_c/resource/srv__type_support_c.cpp.em
// generated code does not contain a copyright notice

@# Included from rosidl_typesupport_opensplice_c/resource/idl__dds_opensplice_type_support.c.em
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
include_parts = [package_name] + list(interface_path.parents[0].parts)
include_dir = '/'.join(include_parts)
include_parts.append(convert_camel_case_to_lower_case_underscore(interface_path.stem))
include_base = '/'.join(include_parts)
header_file = include_base + '__rosidl_typesupport_opensplice_c.h'
}@
@{
service_name = service.namespaced_type.name
header_files = [
    header_file,
    'rmw/rmw.h',
    package_name + '/msg/rosidl_typesupport_opensplice_c__visibility_control.h',
    include_base + '.h',
    include_dir + '/dds_opensplice/ccpp_' + interface_path.stem + '_.h',
    'rosidl_typesupport_opensplice_c/identifier.h',
    # TODO(dirk-thomas) including another cpp file is just nasty
    include_dir + '/dds_opensplice/' + convert_camel_case_to_lower_case_underscore(interface_path.stem) + '__type_support.cpp',
    package_name + '/msg/rosidl_generator_c__visibility_control.h',
]
}@
@[for header_file in header_files]@
@[  if header_file in include_directives]@
// already included above
// @
@[  else]@
@{include_directives.add(header_file)}@
@[  end if]@
#include "@(header_file)"
@[end for]@

@{
TEMPLATE(
    'msg__type_support_c.cpp.em',
    package_name=package_name,
    interface_path=interface_path,
    message=service.request_message,
    include_directives=include_directives,
)
TEMPLATE(
    'msg__type_support_c.cpp.em',
    package_name=package_name,
    interface_path=interface_path,
    message=service.response_message,
    include_directives=include_directives,
)
}@

@{
package_namespaces = '::'.join(service.namespaced_type.namespaces)
__dds_msg_type_prefix = "{ns}::dds_::{typename}".format(ns=package_namespaces, typename=service.namespaced_type.name)
__dds_sample_type_prefix = "{ns}::dds_::Sample_{typename}".format(ns=package_namespaces, typename=service.namespaced_type.name)
}@
#if defined(__cplusplus)
extern "C"
{
#endif

// forward declare type support functions
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_opensplice_c, @(', '.join(service.namespaced_type.namespaced_name()))_Request)();
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_opensplice_c, @(', '.join(service.namespaced_type.namespaced_name()))_Response)();

const char *
register_types__@('__'.join(service.namespaced_type.namespaced_name()))(
  void * untyped_participant, const char * request_type_name, const char * response_type_name)
{
  return @('::'.join(service.namespaced_type.namespaces))::typesupport_opensplice_cpp::register_types__@(service.namespaced_type.name)(
    untyped_participant, request_type_name, response_type_name);
}

const char *
create_requester__@('__'.join(service.namespaced_type.namespaced_name()))(
  void * untyped_participant, const char * service_name,
  void ** untyped_requester, void ** untyped_reader,
  const void * untyped_datareader_qos,
  const void * untyped_datawriter_qos,
  bool avoid_ros_namespace_conventions,
  void * (*allocator)(size_t))
{
  return @('::'.join(service.namespaced_type.namespaces))::typesupport_opensplice_cpp::create_requester__@(service.namespaced_type.name)(
    untyped_participant, service_name,
    untyped_requester, untyped_reader,
    untyped_datareader_qos,
    untyped_datawriter_qos,
    avoid_ros_namespace_conventions,
    allocator);
}

const char *
create_responder__@('__'.join(service.namespaced_type.namespaced_name()))(
  void * untyped_participant, const char * service_name,
  void ** untyped_responder, void ** untyped_reader,
  const void * untyped_datareader_qos,
  const void * untyped_datawriter_qos,
  bool avoid_ros_namespace_conventions,
  void * (*allocator)(size_t))
{
  return @('::'.join(service.namespaced_type.namespaces))::typesupport_opensplice_cpp::create_responder__@(service.namespaced_type.name)(
    untyped_participant, service_name,
    untyped_responder, untyped_reader,
    untyped_datareader_qos,
    untyped_datawriter_qos,
    avoid_ros_namespace_conventions,
    allocator);
}

const char *
send_request__@('__'.join(service.namespaced_type.namespaced_name()))(
  void * untyped_requester, const void * untyped_ros_request, int64_t * sequence_number)
{
  using SampleT = rosidl_typesupport_opensplice_cpp::Sample<@(__dds_msg_type_prefix)_Request_>;

  SampleT request;
  const rosidl_message_type_support_t * ts =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_opensplice_c, @(', '.join(service.namespaced_type.namespaced_name()))_Request)();
  const message_type_support_callbacks_t * callbacks =
    static_cast<const message_type_support_callbacks_t *>(ts->data);
  callbacks->convert_ros_to_dds(untyped_ros_request, static_cast<void *>(&request.data()));

  using RequesterT = rosidl_typesupport_opensplice_cpp::Requester<
    @(__dds_msg_type_prefix)_Request_,
    @(__dds_msg_type_prefix)_Response_
  >;

  auto requester = reinterpret_cast<RequesterT *>(untyped_requester);

  const char * error_string = requester->send_request(request);
  if (error_string) {
    return error_string;
  }
  *sequence_number = request.sequence_number_;

  return nullptr;
}

const char *
take_request__@('__'.join(service.namespaced_type.namespaced_name()))(
  void * untyped_responder, rmw_request_id_t * request_header, void * untyped_ros_request,
  bool * taken)
{
  using ResponderT = rosidl_typesupport_opensplice_cpp::Responder<
    @(__dds_msg_type_prefix)_Request_,
    @(__dds_msg_type_prefix)_Response_
  >;

  auto responder = reinterpret_cast<ResponderT *>(untyped_responder);

  rosidl_typesupport_opensplice_cpp::Sample<@(__dds_msg_type_prefix)_Request_> request;
  const char * error_string = responder->take_request(request, taken);
  if (error_string) {
    return error_string;
  }

  if (*taken) {
    const rosidl_message_type_support_t * ts =
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_opensplice_c, @(', '.join(service.namespaced_type.namespaced_name()))_Request)();
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(ts->data);
    callbacks->convert_dds_to_ros(static_cast<void *>(&request.data()), untyped_ros_request);

    request_header->sequence_number = request.sequence_number_;
    std::memcpy(
      &request_header->writer_guid[0], &request.client_guid_0_, sizeof(request.client_guid_0_));
    std::memcpy(
      &request_header->writer_guid[0] + sizeof(request.client_guid_0_),
      &request.client_guid_1_, sizeof(request.client_guid_1_));

    *taken = true;
    return nullptr;
  }
  *taken = false;
  return nullptr;
}

const char *
send_response__@('__'.join(service.namespaced_type.namespaced_name()))(
  void * untyped_responder, const rmw_request_id_t * request_header,
  const void * untyped_ros_response)
{
  rosidl_typesupport_opensplice_cpp::Sample<@(__dds_msg_type_prefix)_Response_> response;
  const rosidl_message_type_support_t * ts =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_opensplice_c, @(', '.join(service.namespaced_type.namespaced_name()))_Response)();
  const message_type_support_callbacks_t * callbacks =
    static_cast<const message_type_support_callbacks_t *>(ts->data);
  callbacks->convert_ros_to_dds(untyped_ros_response, static_cast<void *>(&response.data()));

  using ResponderT = rosidl_typesupport_opensplice_cpp::Responder<
    @(__dds_msg_type_prefix)_Request_,
    @(__dds_msg_type_prefix)_Response_
  >;
  auto responder = reinterpret_cast<ResponderT *>(untyped_responder);

  const char * error_string = responder->send_response(*request_header, response);
  if (error_string) {
    return error_string;
  }
  return nullptr;
}

const char *
take_response__@('__'.join(service.namespaced_type.namespaced_name()))(
  void * untyped_requester, rmw_request_id_t * request_header, void * untyped_ros_response,
  bool * taken)
{
  using RequesterT = rosidl_typesupport_opensplice_cpp::Requester<
    @(__dds_msg_type_prefix)_Request_,
    @(__dds_msg_type_prefix)_Response_
  >;
  auto requester = reinterpret_cast<RequesterT *>(untyped_requester);

  rosidl_typesupport_opensplice_cpp::Sample<@(__dds_msg_type_prefix)_Response_> response;
  const char * error_string = requester->take_response(response, taken);
  if (error_string) {
    return error_string;
  }
  if (*taken) {
    request_header->sequence_number = response.sequence_number_;

    const rosidl_message_type_support_t * ts =
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_opensplice_c, @(', '.join(service.namespaced_type.namespaced_name()))_Response)();
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(ts->data);
    callbacks->convert_dds_to_ros(
      static_cast<void *>(&response.data()), untyped_ros_response);
    return nullptr;
  }

  return nullptr;
}

const char *
destroy_requester__@('__'.join(service.namespaced_type.namespaced_name()))(void * untyped_requester, void (* deallocator)(void *))
{
  return @('::'.join(service.namespaced_type.namespaces))::typesupport_opensplice_cpp::destroy_requester__@(service.namespaced_type.name)(
    untyped_requester, deallocator);
}

const char *
destroy_responder__@('__'.join(service.namespaced_type.namespaced_name()))(void * untyped_responder, void (* deallocator)(void *))
{
  return @('::'.join(service.namespaced_type.namespaces))::typesupport_opensplice_cpp::destroy_responder__@(service.namespaced_type.name)(
    untyped_responder, deallocator);
}

const char *
server_is_available__@('__'.join(service.namespaced_type.namespaced_name()))(
  void * requester, const rmw_node_t * node, bool * is_available)
{
  return @('::'.join(service.namespaced_type.namespaces))::typesupport_opensplice_cpp::server_is_available__@(service.namespaced_type.name)(
    requester, node, is_available);
}

static service_type_support_callbacks_t @(service.namespaced_type.name)__callbacks = {
  "@('::'.join([package_name] + list(interface_path.parents[0].parts)))",
  "@(service.namespaced_type.name)",
  &create_requester__@('__'.join(service.namespaced_type.namespaced_name())),
  &destroy_requester__@('__'.join(service.namespaced_type.namespaced_name())),
  &create_responder__@('__'.join(service.namespaced_type.namespaced_name())),
  &destroy_responder__@('__'.join(service.namespaced_type.namespaced_name())),
  &send_request__@('__'.join(service.namespaced_type.namespaced_name())),
  &take_request__@('__'.join(service.namespaced_type.namespaced_name())),
  &send_response__@('__'.join(service.namespaced_type.namespaced_name())),
  &take_response__@('__'.join(service.namespaced_type.namespaced_name())),
  &server_is_available__@('__'.join(service.namespaced_type.namespaced_name())),
};

static rosidl_service_type_support_t @(service.namespaced_type.name)__type_support = {
  rosidl_typesupport_opensplice_c__identifier,
  &@(service.namespaced_type.name)__callbacks,  // data
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_opensplice_c, @(', '.join(service.namespaced_type.namespaced_name())))() {
  return &@(service.namespaced_type.name)__type_support;
}

#if defined(__cplusplus)
}
#endif
