// Copyright 2015-2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstring>
#include <string>

#include "rcutils/filesystem.h"

#include "rmw_connext_shared_cpp/guard_condition.hpp"
#include "rmw_connext_shared_cpp/init.hpp"
#include "rmw_connext_shared_cpp/ndds_include.hpp"
#include "rmw_connext_shared_cpp/node.hpp"
#include "rmw_connext_shared_cpp/security_logging.hpp"
#include "rmw_connext_shared_cpp/types.hpp"

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

rmw_node_t *
create_node(
  const char * implementation_identifier,
  rmw_context_t * context,
  const char * name,
  const char * namespace_,
  size_t domain_id,
  bool localhost_only)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, NULL);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init context,
    context->implementation_identifier,
    implementation_identifier,
    // TODO(wjwwood): replace this with RMW_RET_INCORRECT_RMW_IMPLEMENTATION when refactored
    return NULL);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    context->impl,
    "expected initialized context",
    return NULL);
  if (context->impl->is_shutdown) {
    RCUTILS_SET_ERROR_MSG("context has been shutdown");
    return NULL;
  }

  int validation_result = RMW_NODE_NAME_VALID;
  rmw_ret_t ret = rmw_validate_node_name(name, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return NULL;
  }
  if (RMW_NODE_NAME_VALID != validation_result) {
    const char * reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node name: %s", reason);
    return NULL;
  }
  validation_result = RMW_NAMESPACE_VALID;
  ret = rmw_validate_namespace(namespace_, &validation_result, nullptr);
  if (RMW_RET_OK != ret) {
    return NULL;
  }
  if (RMW_NAMESPACE_VALID != validation_result) {
    const char * reason = rmw_node_name_validation_result_string(validation_result);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("invalid node namespace: %s", reason);
    return NULL;
  }

  DDS::DomainParticipantFactory * dpf_ = DDS::DomainParticipantFactory::get_instance();
  if (!dpf_) {
    RMW_SET_ERROR_MSG("failed to get participant factory");
    return NULL;
  }

  // use loopback interface to enable cross vendor communication
  DDS::DomainParticipantQos participant_qos;
  DDS::ReturnCode_t status = dpf_->get_default_participant_qos(participant_qos);
  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get default participant qos");
    return NULL;
  }

  if (localhost_only) {
    status = DDS::PropertyQosPolicyHelper::add_property(
      participant_qos.property,
      "dds.transport.UDPv4.builtin.parent.allow_interfaces",
      "127.0.0.1",
      DDS::BOOLEAN_FALSE);
    if (status != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG(
        "failed to add qos property to set localhost as the only network interface");
      return NULL;
    }
  }
  // This String_dup is not matched with a String_free because DDS appears to
  // free this automatically.
  participant_qos.participant_name.name = DDS::String_dup(name);
  // since the participant name is not part of the DDS spec
  // the node name is also set in the user_data
  size_t length = std::snprintf(
    nullptr,
    0,
    "name=%s;namespace=%s;enclave=%s;",
    name, namespace_, context->options.enclave) + 1;
  bool success = participant_qos.user_data.value.length(static_cast<DDS::Long>(length));
  if (!success) {
    RMW_SET_ERROR_MSG("failed to resize participant user_data");
    return NULL;
  }

  int written = std::snprintf(
    reinterpret_cast<char *>(participant_qos.user_data.value.get_contiguous_buffer()),
    length,
    "name=%s;namespace=%s;enclave=%s;",
    name, namespace_, context->options.enclave);
  if (written < 0 || written > static_cast<int>(length) - 1) {
    RMW_SET_ERROR_MSG("failed to populate user_data buffer");
    return NULL;
  }

  // According to the RTPS spec, ContentFilterProperty_t has the following fields:
  // -contentFilteredTopicName (max length 256)
  // -relatedTopicName (max length 256)
  // -filterClassName (max length 256)
  // -filterName (DDSSQL)
  // -filterExpression
  // In Connext, contentfilter_property_max_length is sum of lengths of all these fields,
  // which by default is 256.
  // So we set the limit to 1024, to accomodate the complete topic name with namespaces.
  participant_qos.resource_limits.contentfilter_property_max_length = 1024;

  // forces local traffic to be sent over loopback,
  // even if a more efficient transport (such as shared memory) is installed
  // (in which case traffic will be sent over both transports)
  status = DDS::PropertyQosPolicyHelper::add_property(
    participant_qos.property,
    "dds.transport.UDPv4.builtin.ignore_loopback_interface",
    "0",
    DDS::BOOLEAN_FALSE);
  if (status != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to add qos property");
    return NULL;
  }

  // Disable TypeCode since it increases discovery message size and is replaced by TypeObject
  // https://community.rti.com/kb/types-matching
  participant_qos.resource_limits.type_code_max_serialized_length = 0;

  rmw_node_t * node_handle = nullptr;
  ConnextNodeInfo * node_info = nullptr;
  rmw_guard_condition_t * graph_guard_condition = nullptr;
  CustomPublisherListener * publisher_listener = nullptr;
  CustomSubscriberListener * subscriber_listener = nullptr;
  void * buf = nullptr;

  DDS::DomainParticipant * participant = nullptr;
  DDS::DataReader * data_reader = nullptr;
  DDS::PublicationBuiltinTopicDataDataReader * builtin_publication_datareader = nullptr;
  DDS::SubscriptionBuiltinTopicDataDataReader * builtin_subscription_datareader = nullptr;
  DDS::Subscriber * builtin_subscriber = nullptr;

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  const char * srp = nullptr;
  char * identity_ca_cert_fn = nullptr;
  char * permissions_ca_cert_fn = nullptr;
  char * cert_fn = nullptr;
  char * key_fn = nullptr;
  char * gov_fn = nullptr;
  char * perm_fn = nullptr;

  if (context->options.security_options.security_root_path) {
    // enable some security stuff
    status = DDS::PropertyQosPolicyHelper::add_property(
      participant_qos.property,
      "com.rti.serv.load_plugin",
      "com.rti.serv.secure",
      DDS::BOOLEAN_FALSE);
    if (status != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to add security property");
      return NULL;
    }
    status = DDS::PropertyQosPolicyHelper::add_property(
      participant_qos.property,
      "com.rti.serv.secure.library",
      "nddssecurity",
      DDS::BOOLEAN_FALSE);
    if (status != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to add security property");
      return NULL;
    }
    status = DDS::PropertyQosPolicyHelper::add_property(
      participant_qos.property,
      "com.rti.serv.secure.create_function",
      "RTI_Security_PluginSuite_create",
      DDS::BOOLEAN_FALSE);
    if (status != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to add security property");
      return NULL;
    }

    srp = context->options.security_options.security_root_path;  // save some typing
    identity_ca_cert_fn = rcutils_join_path(srp, "identity_ca.cert.pem", allocator);
    if (!identity_ca_cert_fn) {
      RMW_SET_ERROR_MSG("failed to allocate memory for 'identity_ca_cert_fn'");
      goto fail;
    }
    permissions_ca_cert_fn = rcutils_join_path(srp, "permissions_ca.cert.pem", allocator);
    if (!permissions_ca_cert_fn) {
      RMW_SET_ERROR_MSG("failed to allocate memory for 'permissions_ca_cert_fn'");
      goto fail;
    }
    cert_fn = rcutils_join_path(srp, "cert.pem", allocator);
    if (!cert_fn) {
      RMW_SET_ERROR_MSG("failed to allocate memory for 'cert_fn'");
      goto fail;
    }
    key_fn = rcutils_join_path(srp, "key.pem", allocator);
    if (!key_fn) {
      RMW_SET_ERROR_MSG("failed to allocate memory for 'key_fn'");
      goto fail;
    }
    gov_fn = rcutils_join_path(srp, "governance.p7s", allocator);
    if (!gov_fn) {
      RMW_SET_ERROR_MSG("failed to allocate memory for 'gov_fn'");
      goto fail;
    }
    perm_fn = rcutils_join_path(srp, "permissions.p7s", allocator);
    if (!perm_fn) {
      RMW_SET_ERROR_MSG("failed to allocate memory for 'perm_fn'");
      goto fail;
    }

    // now try to pass these filenames to the Authentication plugin
    status = DDS::PropertyQosPolicyHelper::add_property(
      participant_qos.property,
      "com.rti.serv.secure.authentication.ca_file",
      identity_ca_cert_fn,
      DDS::BOOLEAN_FALSE);
    if (status != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to add security property");
      goto fail;
    }
    status = DDS::PropertyQosPolicyHelper::add_property(
      participant_qos.property,
      "com.rti.serv.secure.authentication.certificate_file",
      cert_fn,
      DDS::BOOLEAN_FALSE);
    if (status != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to add security property");
      goto fail;
    }
    status = DDS::PropertyQosPolicyHelper::add_property(
      participant_qos.property,
      "com.rti.serv.secure.authentication.private_key_file",
      key_fn,
      DDS::BOOLEAN_FALSE);
    if (status != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to add security property");
      goto fail;
    }

    // pass filenames to the Access Control plugin
    status = DDS::PropertyQosPolicyHelper::add_property(
      participant_qos.property,
      "com.rti.serv.secure.access_control.permissions_authority_file",
      permissions_ca_cert_fn,
      DDS::BOOLEAN_FALSE);
    if (status != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to add security property");
      goto fail;
    }

    status = DDS::PropertyQosPolicyHelper::add_property(
      participant_qos.property,
      "com.rti.serv.secure.access_control.governance_file",
      gov_fn,
      DDS::BOOLEAN_FALSE);
    if (status != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to add security property");
      goto fail;
    }

    status = DDS::PropertyQosPolicyHelper::add_property(
      participant_qos.property,
      "com.rti.serv.secure.access_control.permissions_file",
      perm_fn,
      DDS::BOOLEAN_FALSE);
    if (status != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG("failed to add security property");
      goto fail;
    }

    // Configure security logging
    if (apply_security_logging_configuration(participant_qos.property) != RMW_RET_OK) {
      goto fail;
    }
  }

  // No custom handling of RMW_DEFAULT_DOMAIN_ID. Simply use a reasonable domain id.
  participant = dpf_->create_participant(
    static_cast<DDS::DomainId_t>(
      domain_id != RMW_DEFAULT_DOMAIN_ID ? domain_id : 0u),
    participant_qos,
    NULL,
    DDS::STATUS_MASK_NONE);
  if (!participant) {
    RMW_SET_ERROR_MSG("failed to create participant");
    goto fail;
  }

  builtin_subscriber = participant->get_builtin_subscriber();
  if (!builtin_subscriber) {
    RMW_SET_ERROR_MSG("builtin subscriber handle is null");
    goto fail;
  }

  // setup publisher listener
  data_reader = builtin_subscriber->lookup_datareader(DDS::PUBLICATION_TOPIC_NAME);
  builtin_publication_datareader =
    static_cast<DDS::PublicationBuiltinTopicDataDataReader *>(data_reader);
  if (!builtin_publication_datareader) {
    RMW_SET_ERROR_MSG("builtin publication datareader handle is null");
    goto fail;
  }

  graph_guard_condition = create_guard_condition(implementation_identifier, context);
  if (!graph_guard_condition) {
    RMW_SET_ERROR_MSG("failed to create graph guard condition");
    goto fail;
  }

  buf = rmw_allocate(sizeof(CustomPublisherListener));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  RMW_TRY_PLACEMENT_NEW(
    publisher_listener, buf, goto fail, CustomPublisherListener,
    implementation_identifier, graph_guard_condition)
  buf = nullptr;
  builtin_publication_datareader->set_listener(publisher_listener, DDS::DATA_AVAILABLE_STATUS);

  data_reader = builtin_subscriber->lookup_datareader(DDS::SUBSCRIPTION_TOPIC_NAME);
  builtin_subscription_datareader =
    static_cast<DDS::SubscriptionBuiltinTopicDataDataReader *>(data_reader);
  if (!builtin_subscription_datareader) {
    RMW_SET_ERROR_MSG("builtin subscription datareader handle is null");
    goto fail;
  }

  // setup subscriber listener
  buf = rmw_allocate(sizeof(CustomSubscriberListener));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  RMW_TRY_PLACEMENT_NEW(
    subscriber_listener, buf, goto fail, CustomSubscriberListener,
    implementation_identifier, graph_guard_condition)
  buf = nullptr;
  builtin_subscription_datareader->set_listener(subscriber_listener, DDS::DATA_AVAILABLE_STATUS);

  node_handle = rmw_node_allocate();
  if (!node_handle) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node handle");
    goto fail;
  }
  node_handle->implementation_identifier = implementation_identifier;
  node_handle->data = participant;

  node_handle->name =
    reinterpret_cast<const char *>(rmw_allocate(sizeof(char) * strlen(name) + 1));
  if (!node_handle->name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(node_handle->name), name, strlen(name) + 1);

  node_handle->namespace_ =
    reinterpret_cast<const char *>(rmw_allocate(sizeof(char) * strlen(namespace_) + 1));
  if (!node_handle->namespace_) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node namespace");
    goto fail;
  }
  memcpy(const_cast<char *>(node_handle->namespace_), namespace_, strlen(namespace_) + 1);

  buf = rmw_allocate(sizeof(ConnextNodeInfo));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  RMW_TRY_PLACEMENT_NEW(node_info, buf, goto fail, ConnextNodeInfo, )
  buf = nullptr;
  node_info->participant = participant;
  node_info->publisher_listener = publisher_listener;
  node_info->subscriber_listener = subscriber_listener;
  node_info->graph_guard_condition = graph_guard_condition;

  node_handle->implementation_identifier = implementation_identifier;
  node_handle->data = node_info;
  node_handle->context = context;
  return node_handle;
fail:
  status = dpf_->delete_participant(participant);
  if (status != DDS::RETCODE_OK) {
    std::stringstream ss;
    ss << "leaking participant while handling failure at " <<
      __FILE__ << ":" << __LINE__;
    (std::cerr << ss.str()).flush();
  }
  if (graph_guard_condition) {
    rmw_ret_t ret = destroy_guard_condition(implementation_identifier, graph_guard_condition);
    if (ret != RMW_RET_OK) {
      std::stringstream ss;
      ss << "failed to destroy guard condition while handling failure at " <<
        __FILE__ << ":" << __LINE__;
      (std::cerr << ss.str()).flush();
    }
  }
  if (publisher_listener) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      publisher_listener->~CustomPublisherListener(), CustomPublisherListener)
    rmw_free(publisher_listener);
  }
  if (subscriber_listener) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      subscriber_listener->~CustomSubscriberListener(), CustomSubscriberListener)
    rmw_free(subscriber_listener);
  }
  if (node_handle) {
    if (node_handle->name) {
      rmw_free(const_cast<char *>(node_handle->name));
    }
    if (node_handle->namespace_) {
      rmw_free(const_cast<char *>(node_handle->namespace_));
    }
    rmw_free(node_handle);
  }
  if (node_info) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      node_info->~ConnextNodeInfo(), ConnextNodeInfo)
    rmw_free(node_info);
  }
  if (buf) {
    rmw_free(buf);
  }
  // Note: allocator.deallocate(nullptr, ...); is allowed.
  allocator.deallocate(identity_ca_cert_fn, allocator.state);
  allocator.deallocate(permissions_ca_cert_fn, allocator.state);
  allocator.deallocate(cert_fn, allocator.state);
  allocator.deallocate(key_fn, allocator.state);
  allocator.deallocate(gov_fn, allocator.state);
  allocator.deallocate(perm_fn, allocator.state);
  return NULL;
}

rmw_ret_t
destroy_node(const char * implementation_identifier, rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier,
    implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rmw_ret_t ret = RMW_RET_OK;
  auto node_info = static_cast<ConnextNodeInfo *>(node->data);
  auto participant = static_cast<DDS::DomainParticipant *>(node_info->participant);

  // This unregisters types and destroys topics which were shared between
  // publishers and subscribers and could not be cleaned up in the delete functions.
  if (participant->delete_contained_entities() == DDS::RETCODE_OK) {
    DDS::DomainParticipantFactory * dpf_ = DDS::DomainParticipantFactory::get_instance();
    if (dpf_) {
      // To delete a participant, all domain entities must have been already deleted.
      DDS::ReturnCode_t local_ret = dpf_->delete_participant(participant);
      if (DDS::RETCODE_OK != local_ret) {
        RMW_SET_ERROR_MSG("failed to delete participant");
        ret = RMW_RET_ERROR;
      }
    } else {
      RMW_SET_ERROR_MSG("failed to get participant factory");
      ret = RMW_RET_ERROR;
    }
  } else {
    RMW_SET_ERROR_MSG("failed to delete contained entities of participant");
    ret = RMW_RET_ERROR;
  }

  RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
    node_info->publisher_listener->~CustomPublisherListener(), CustomPublisherListener);
  rmw_free(node_info->publisher_listener);

  RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
    node_info->subscriber_listener->~CustomSubscriberListener(), CustomSubscriberListener);
  rmw_free(node_info->subscriber_listener);

  rmw_ret_t local_ret =
    destroy_guard_condition(implementation_identifier, node_info->graph_guard_condition);
  if (local_ret != RMW_RET_OK) {
    if (ret != RMW_RET_OK) {
      RMW_SAFE_FWRITE_TO_STDERR(
        "Failed to delete graph guard condition after function: '"
        RCUTILS_STRINGIFY(__function__) "' failed.\n");
    } else {
      RMW_SET_ERROR_MSG("failed to delete graph guard condition");
      ret = RMW_RET_ERROR;
    }
  }

  rmw_free(node_info);
  rmw_free(const_cast<char *>(node->name));
  rmw_free(const_cast<char *>(node->namespace_));
  rmw_node_free(node);
  return ret;
}

RMW_CONNEXT_SHARED_CPP_PUBLIC
const rmw_guard_condition_t *
node_get_graph_guard_condition(const rmw_node_t * node)
{
  // node argument is checked in calling function.

  ConnextNodeInfo * node_info = static_cast<ConnextNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return nullptr;
  }

  return node_info->graph_guard_condition;
}
