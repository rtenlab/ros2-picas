// Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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

#ifdef __clang__
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wmismatched-tags"
#endif
#if defined(_MSC_VER)
# pragma warning(push)
# pragma warning(disable: 4099)
#endif
#include <ccpp_dds_dcps.h>
#if defined(_MSC_VER)
# pragma warning(pop)
#endif
#ifdef __clang__
# pragma GCC diagnostic pop
#endif
#include <dds_dcps.h>

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"

#include "event_converter.hpp"
#include "identifier.hpp"
#include "types.hpp"

// The extern "C" here enforces that overloading is not used.
extern "C"
{
rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name, const char * namespace_, size_t domain_id,
  const rmw_node_security_options_t * security_options, bool localhost_only)
{
  static_cast<void>(localhost_only);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init context,
    context->implementation_identifier,
    opensplice_cpp_identifier,
    // TODO(wjwwood): replace this with RMW_RET_INCORRECT_RMW_IMPLEMENTATION when refactored
    return nullptr);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, nullptr);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(namespace_, nullptr);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(security_options, nullptr);
  if (security_options->enforce_security) {
    RMW_SET_ERROR_MSG("OpenSplice doesn't support DDS Security");
    return nullptr;
  }
  DDS::DomainParticipantFactory_var dp_factory = DDS::DomainParticipantFactory::get_instance();
  if (!dp_factory) {
    RMW_SET_ERROR_MSG("failed to get domain participant factory");
    return nullptr;
  }
  DDS::DomainParticipantFactoryQos qos;
  if (dp_factory->get_qos(qos) != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to get domain participant factory qos");
    return nullptr;
  }
  // Create entities in the disabled state.
  qos.entity_factory.autoenable_created_entities = false;
  if (dp_factory->set_qos(qos) != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to set domain participant factory qos");
    return nullptr;
  }

  DDS::DomainId_t domain = static_cast<DDS::DomainId_t>(domain_id);
  DDS::DomainParticipant * participant = nullptr;

  // Make sure that the OSPL_URI is set, otherwise node creation will fail.
  char * ospl_uri = nullptr;
  const char * ospl_uri_env = "OSPL_URI";
#ifndef _WIN32
  ospl_uri = getenv(ospl_uri_env);
#else
  size_t ospl_uri_size;
  _dupenv_s(&ospl_uri, &ospl_uri_size, ospl_uri_env);
#endif
  if (!ospl_uri) {
    RMW_SET_ERROR_MSG("OSPL_URI not set");
    return nullptr;
  } else {
#ifdef _WIN32
    free(ospl_uri);
#endif
  }

  // Ensure the ROS_DOMAIN_ID env variable is set, otherwise parsing of the config may fail.
  // Also make sure the it is set to the domain_id passed in, otherwise it will fail.
  // But first backup the current ROS_DOMAIN_ID.
  char * ros_domain_id = nullptr;
  const char * env_var = "ROS_DOMAIN_ID";
#ifndef _WIN32
  ros_domain_id = getenv(env_var);
#else
  size_t ros_domain_id_size;
  _dupenv_s(&ros_domain_id, &ros_domain_id_size, env_var);
#endif

  // On Windows, setting the ROS_DOMAIN_ID does not fix the problem, so error early.
#ifdef _WIN32
  if (!ros_domain_id) {
    RMW_SET_ERROR_MSG("environment variable ROS_DOMAIN_ID is not set");
    fprintf(stderr, "[rmw_opensplice_cpp]: error: %s\n", rmw_get_error_string().str);
    return nullptr;
  }
#endif

  // Set the ROS_DOMAIN_ID explicitly (if not Windows).
#ifndef _WIN32
  auto domain_id_as_string = std::to_string(domain_id);
  int ret = 0;
  ret = setenv(env_var, domain_id_as_string.c_str(), 1);
  if (0 != ret) {
    RMW_SET_ERROR_MSG("failed to set the ROS_DOMAIN_ID");
    return nullptr;
  }
#endif

  // Communicate the ROS Node name as DDS Builtin Participant Topic userData
  //   For extensibility and backwards compatibility a format is applied that must be understood.
  //   The format specifies a list of name-value pairs,
  //   Format (byte ascii string) syntax : { <name> '=' <value> ';' }*
  //   A <name> is a string and a <value> is a sequence of octets.
  //   The '=' and ';' characters are reserved delimiters.
  //   For <name> only alphanumeric character are allowed.
  //   For <value> there is no limitation, ';' can be used as part of the <value> when escaped by
  //   a second ';'.
  //   Implemented policy value: "name=<node-name>;"
  //   userData not following this policy will be ignored completely.

  DDS::DomainParticipantQos dpqos;
  size_t length = strlen(name) + strlen("name=;") +
    strlen(namespace_) + strlen("namespace=;") + 1;
  dp_factory->get_default_participant_qos(dpqos);
  dpqos.user_data.value.length(static_cast<DDS::ULong>(length));
  int written = snprintf(reinterpret_cast<char *>(dpqos.user_data.value.get_buffer(false)),
      length, "name=%s;namespace=%s;", name, namespace_);
  if (written < 0 || written > static_cast<int>(length) - 1) {
    RMW_SET_ERROR_MSG("failed to populate user_data buffer");
    return nullptr;
  }

  participant = dp_factory->create_participant(
    domain, dpqos, NULL, DDS::STATUS_MASK_NONE);
  if (!participant) {
    RMW_SET_ERROR_MSG("failed to create domain participant");
    return NULL;
  }

  // Restore the ROS_DOMAIN_ID if necessary (and not Windows).
#ifndef _WIN32
  if (ros_domain_id) {
    ret = setenv(env_var, ros_domain_id, 1);
    if (0 != ret) {
      RMW_SET_ERROR_MSG("failed to reset the ROS_DOMAIN_ID");
      return nullptr;
    }
  } else {
    // Otherwise unset it again.
    ret = unsetenv(env_var);
    if (0 != ret) {
      RMW_SET_ERROR_MSG("failed to unset the ROS_DOMAIN_ID");
      return nullptr;
    }
  }
#endif

  // enable the participant, otherwise get_builtin_subscriber can't be called
  if (participant->enable() != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to enable domain participant");
    return nullptr;
  }

  rmw_node_t * node = nullptr;
  OpenSpliceStaticNodeInfo * node_info = nullptr;
  rmw_guard_condition_t * graph_guard_condition = nullptr;
  CustomPublisherListener * publisher_listener = nullptr;
  CustomSubscriberListener * subscriber_listener = nullptr;
  void * buf = nullptr;

  DDS::DataReader * data_reader = nullptr;
  DDS::PublicationBuiltinTopicDataDataReader * builtin_publication_datareader = nullptr;
  DDS::SubscriptionBuiltinTopicDataDataReader * builtin_subscription_datareader = nullptr;
  DDS::Subscriber * builtin_subscriber = participant->get_builtin_subscriber();
  if (!builtin_subscriber) {
    RMW_SET_ERROR_MSG("builtin subscriber handle is null");
    goto fail;
  }

  graph_guard_condition = rmw_create_guard_condition(context);
  if (!graph_guard_condition) {
    // error message already set
    goto fail;
  }

  // setup publisher listener
  data_reader = builtin_subscriber->lookup_datareader("DCPSPublication");
  builtin_publication_datareader =
    DDS::PublicationBuiltinTopicDataDataReader::_narrow(data_reader);
  if (!builtin_publication_datareader) {
    RMW_SET_ERROR_MSG("builtin publication datareader handle is null");
    goto fail;
  }

  buf = rmw_allocate(sizeof(CustomPublisherListener));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  RMW_TRY_PLACEMENT_NEW(
    publisher_listener, buf, goto fail, CustomPublisherListener, graph_guard_condition)
  buf = nullptr;
  builtin_publication_datareader->set_listener(publisher_listener, DDS::DATA_AVAILABLE_STATUS);
  // listener is set on an already created reader. This could mean that the reader already contains
  // data for which the on_data_available callback is not called. To ensure we have not missed
  // anything manually call the on_data_available callback.
  publisher_listener->on_data_available(builtin_publication_datareader);

  data_reader = builtin_subscriber->lookup_datareader("DCPSSubscription");
  builtin_subscription_datareader =
    DDS::SubscriptionBuiltinTopicDataDataReader::_narrow(data_reader);
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
    subscriber_listener, buf, goto fail, CustomSubscriberListener, graph_guard_condition)
  buf = nullptr;
  builtin_subscription_datareader->set_listener(subscriber_listener, DDS::DATA_AVAILABLE_STATUS);
  // listener is set on an already created reader. This could mean that the reader already contains
  // data for which the on_data_available callback is not called. To ensure we have not missed
  // anything manually call the on_data_available callback.
  subscriber_listener->on_data_available(builtin_subscription_datareader);

  node = rmw_node_allocate();
  if (!node) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_node_t");
    goto fail;
  }

  node->name = reinterpret_cast<const char *>(rmw_allocate(sizeof(char) * strlen(name) + 1));
  if (!node->name) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node name");
    goto fail;
  }
  memcpy(const_cast<char *>(node->name), name, strlen(name) + 1);

  node->namespace_ = reinterpret_cast<const char *>(
    rmw_allocate(sizeof(char) * strlen(namespace_) + 1));
  if (!node->namespace_) {
    RMW_SET_ERROR_MSG("failed to allocate memory for node namespace");
    goto fail;
  }
  memcpy(const_cast<char *>(node->namespace_), namespace_, strlen(namespace_) + 1);

  buf = rmw_allocate(sizeof(OpenSpliceStaticNodeInfo));
  if (!buf) {
    RMW_SET_ERROR_MSG("failed to allocate memory");
    goto fail;
  }
  RMW_TRY_PLACEMENT_NEW(node_info, buf, goto fail, OpenSpliceStaticNodeInfo, )
  buf = nullptr;
  node_info->participant = participant;
  node_info->graph_guard_condition = graph_guard_condition;
  node_info->publisher_listener = publisher_listener;
  node_info->subscriber_listener = subscriber_listener;

  node->implementation_identifier = opensplice_cpp_identifier;
  node->data = node_info;

  return node;
fail:
  if (participant) {
    if (dp_factory->delete_participant(participant) != DDS::RETCODE_OK) {
      std::stringstream ss;
      ss << "leaking domain participant while handling failure at: " <<
        __FILE__ << ":" << __LINE__ << '\n';
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
  if (graph_guard_condition) {
    rmw_ret_t ret = rmw_destroy_guard_condition(graph_guard_condition);
    if (ret != RMW_RET_OK) {
      fprintf(stderr, "failed to destroy guard condition: %s\n", rmw_get_error_string().str);
    }
  }
  if (node_info) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      node_info->~OpenSpliceStaticNodeInfo(), OpenSpliceStaticNodeInfo)
    rmw_free(node_info);
  }
  if (buf) {
    rmw_free(buf);
  }
  if (node) {
    if (node->name) {
      rmw_free(const_cast<char *>(node->name));
    }
    if (node->namespace_) {
      rmw_free(const_cast<char *>(node->namespace_));
    }
    rmw_node_free(node);
  }
  return nullptr;
}

rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  if (!node) {
    RMW_SET_ERROR_MSG("received null pointer");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, opensplice_cpp_identifier,
    return RMW_RET_ERROR)

  DDS::DomainParticipantFactory_var dp_factory = DDS::DomainParticipantFactory::get_instance();
  if (!dp_factory) {
    RMW_SET_ERROR_MSG("failed to get domain participant factory");
    return RMW_RET_ERROR;
  }
  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return RMW_RET_ERROR;
  }
  auto participant = static_cast<DDS::DomainParticipant *>(node_info->participant);
  if (!participant) {
    RMW_SET_ERROR_MSG("participant handle is null");
    return RMW_RET_ERROR;
  }

  auto result = RMW_RET_OK;

  // Explicitly delete the builtin_subscriber, which is
  // apparently required because we accessed it in rmw_create_node().
  DDS::Subscriber * builtin_subscriber = participant->get_builtin_subscriber();
  if (builtin_subscriber) {
    if (participant->delete_subscriber(builtin_subscriber) != DDS::RETCODE_OK) {
      RMW_SET_ERROR_MSG("builtin subscriber handle failed to delete");
      result = RMW_RET_ERROR;
    }
  }

  // This unregisters types and destroys topics which were shared between
  // publishers and subscribers and could not be cleaned up in the delete functions.
  if (participant->delete_contained_entities() != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to delete contained entities of participant");
    result = RMW_RET_ERROR;
  }

  if (dp_factory->delete_participant(participant) != DDS::RETCODE_OK) {
    RMW_SET_ERROR_MSG("failed to delete participant");
    result = RMW_RET_ERROR;
  }

  if (node_info->publisher_listener) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      node_info->publisher_listener->~CustomPublisherListener(), CustomPublisherListener)
    rmw_free(node_info->publisher_listener);
    node_info->publisher_listener = nullptr;
  }
  if (node_info->subscriber_listener) {
    RMW_TRY_DESTRUCTOR_FROM_WITHIN_FAILURE(
      node_info->subscriber_listener->~CustomSubscriberListener(), CustomSubscriberListener)
    rmw_free(node_info->subscriber_listener);
    node_info->subscriber_listener = nullptr;
  }

  if (node_info->graph_guard_condition) {
    rmw_ret_t ret = rmw_destroy_guard_condition(node_info->graph_guard_condition);
    if (ret != RMW_RET_OK) {
      fprintf(stderr, "failed to destroy guard condition: %s\n", rmw_get_error_string().str);
    }
  }

  rmw_free(node_info);
  node->data = nullptr;
  rmw_free(const_cast<char *>(node->name));
  node->name = nullptr;
  rmw_free(const_cast<char *>(node->namespace_));
  node->namespace_ = nullptr;
  rmw_node_free(node);
  return result;
}

rmw_ret_t
rmw_node_assert_liveliness(const rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);

  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  if (nullptr == node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return RMW_RET_ERROR;
  }
  if (nullptr == node_info->participant) {
    RMW_SET_ERROR_MSG("node internal participant is invalid");
    return RMW_RET_ERROR;
  }

  rmw_ret_t result = check_dds_ret_code(node_info->participant->assert_liveliness());
  if (RMW_RET_OK != result) {
    RMW_SET_ERROR_MSG("failed to assert liveliness of participant");
  }
  return result;
}

const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(const rmw_node_t * node)
{
  if (!node) {
    RMW_SET_ERROR_MSG("received null pointer");
    return NULL;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node handle,
    node->implementation_identifier, opensplice_cpp_identifier,
    return NULL)
  auto node_info = static_cast<OpenSpliceStaticNodeInfo *>(node->data);
  if (!node_info) {
    RMW_SET_ERROR_MSG("node info handle is null");
    return NULL;
  }
  return node_info->graph_guard_condition;
}
}  // extern "C"
