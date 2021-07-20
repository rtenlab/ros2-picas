// Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

#include <cstddef>
#include <cstdio>
#include <cstring>

#include <list>
#include <string>

#include <fstream>
#include <sstream>

#include "rcutils/allocator.h"
#include "rcutils/format_string.h"
#include "rcutils/types/string_array.h"

#include "Poco/SharedLibrary.h"

#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/names_and_types.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/rmw.h"

std::string get_env_var(const char * env_var)
{
  char * value = nullptr;
#ifndef _WIN32
  value = getenv(env_var);
#else
  size_t value_size;
  _dupenv_s(&value, &value_size, env_var);
#endif
  std::string value_str = "";
  if (value) {
    value_str = value;
#ifdef _WIN32
    free(value);
#endif
  }
  // printf("get_env_var(%s) = %s\n", env_var, value_str.c_str());
  return value_str;
}

std::list<std::string> split(const std::string & value, const char delimiter)
{
  std::list<std::string> list;
  std::istringstream ss(value);
  std::string s;
  while (std::getline(ss, s, delimiter)) {
    list.push_back(s);
  }
  // printf("split(%s) = %zu\n", value.c_str(), list.size());
  return list;
}

bool is_file_exist(const char * filename)
{
  std::ifstream h(filename);
  // printf("is_file_exist(%s) = %s\n", filename, h.good() ? "true" : "false");
  return h.good();
}

std::string find_library_path(const std::string & library_name)
{
  const char * env_var;
  char separator;
  const char * filename_prefix;
  const char * filename_extension;
#ifdef _WIN32
  env_var = "PATH";
  separator = ';';
  filename_prefix = "";
  filename_extension = ".dll";
#elif __APPLE__
  env_var = "DYLD_LIBRARY_PATH";
  separator = ':';
  filename_prefix = "lib";
  filename_extension = ".dylib";
#else
  env_var = "LD_LIBRARY_PATH";
  separator = ':';
  filename_prefix = "lib";
  filename_extension = ".so";
#endif
  std::string search_path = get_env_var(env_var);
  std::list<std::string> search_paths = split(search_path, separator);

  std::string filename = filename_prefix;
  filename += library_name + filename_extension;

  for (auto it : search_paths) {
    std::string path = it + "/" + filename;
    if (is_file_exist(path.c_str())) {
      return path;
    }
  }
  return "";
}

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

Poco::SharedLibrary *
get_library()
{
  static Poco::SharedLibrary * lib = nullptr;
  if (!lib) {
    std::string env_var = get_env_var("RMW_IMPLEMENTATION");
    if (env_var.empty()) {
      env_var = STRINGIFY(DEFAULT_RMW_IMPLEMENTATION);
    }
    std::string library_path = find_library_path(env_var);
    if (library_path.empty()) {
      RMW_SET_ERROR_MSG(
        ("failed to find shared library of rmw implementation. Searched " + env_var).c_str());
      return nullptr;
    }
    try {
      lib = new Poco::SharedLibrary(library_path);
    } catch (Poco::LibraryLoadException & e) {
      RMW_SET_ERROR_MSG(("failed to load shared library of rmw implementation. Exception: " +
        e.displayText()).c_str());
      return nullptr;
    } catch (...) {
      RMW_SET_ERROR_MSG("failed to load shared library of rmw implementation");
      RMW_SET_ERROR_MSG(
        ("failed to load shared library of rmw implementation: " + library_path).c_str());
      return nullptr;
    }
  }
  return lib;
}

void *
get_symbol(const char * symbol_name)
{
  Poco::SharedLibrary * lib = get_library();
  if (!lib) {
    // error message set by get_library()
    return nullptr;
  }
  if (!lib->hasSymbol(symbol_name)) {
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    char * msg = rcutils_format_string(
      allocator,
      "failed to resolve symbol '%s' in shared library '%s'", symbol_name, lib->getPath().c_str());
    if (msg) {
      RMW_SET_ERROR_MSG(msg);
      allocator.deallocate(msg, allocator.state);
    } else {
      RMW_SET_ERROR_MSG("failed to allocate memory for error message");
    }
    return nullptr;
  }
  return lib->getSymbol(symbol_name);
}

#ifdef __cplusplus
extern "C"
{
#endif

#define EXPAND(x) x

#define ARG_TYPES(...) __VA_ARGS__

#define ARG_VALUES_0(...)
#define ARG_VALUES_1(t1) v1
#define ARG_VALUES_2(t2, ...) v2, EXPAND(ARG_VALUES_1(__VA_ARGS__))
#define ARG_VALUES_3(t3, ...) v3, EXPAND(ARG_VALUES_2(__VA_ARGS__))
#define ARG_VALUES_4(t4, ...) v4, EXPAND(ARG_VALUES_3(__VA_ARGS__))
#define ARG_VALUES_5(t5, ...) v5, EXPAND(ARG_VALUES_4(__VA_ARGS__))
#define ARG_VALUES_6(t6, ...) v6, EXPAND(ARG_VALUES_5(__VA_ARGS__))
#define ARG_VALUES_7(t7, ...) v7, EXPAND(ARG_VALUES_6(__VA_ARGS__))

#define ARGS_0(...) __VA_ARGS__
#define ARGS_1(t1) t1 v1
#define ARGS_2(t2, ...) t2 v2, EXPAND(ARGS_1(__VA_ARGS__))
#define ARGS_3(t3, ...) t3 v3, EXPAND(ARGS_2(__VA_ARGS__))
#define ARGS_4(t4, ...) t4 v4, EXPAND(ARGS_3(__VA_ARGS__))
#define ARGS_5(t5, ...) t5 v5, EXPAND(ARGS_4(__VA_ARGS__))
#define ARGS_6(t6, ...) t6 v6, EXPAND(ARGS_5(__VA_ARGS__))
#define ARGS_7(t7, ...) t7 v7, EXPAND(ARGS_6(__VA_ARGS__))

#define CALL_SYMBOL(symbol_name, ReturnType, error_value, ArgTypes, arg_values) \
  if (!symbol_ ## symbol_name) { \
    /* only necessary for functions called before rmw_init */ \
    symbol_ ## symbol_name = get_symbol(#symbol_name); \
  } \
  if (!symbol_ ## symbol_name) { \
    /* error message set by get_symbol() */ \
    return error_value; \
  } \
  typedef ReturnType (* FunctionSignature)(ArgTypes); \
  FunctionSignature func = reinterpret_cast<FunctionSignature>(symbol_ ## symbol_name); \
  return func(arg_values);

// cppcheck-suppress preprocessorErrorDirective
#define RMW_INTERFACE_FN(name, ReturnType, error_value, _NR, ...) \
  void * symbol_ ## name = nullptr; \
  ReturnType name(EXPAND(ARGS_ ## _NR(__VA_ARGS__))) \
  { \
    CALL_SYMBOL(name, ReturnType, error_value, ARG_TYPES(__VA_ARGS__), \
      EXPAND(ARG_VALUES_ ## _NR(__VA_ARGS__))); \
  }

RMW_INTERFACE_FN(rmw_get_implementation_identifier,
  const char *, nullptr,
  0, ARG_TYPES(void))

RMW_INTERFACE_FN(rmw_init_options_init,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(rmw_init_options_t *, rcutils_allocator_t))

RMW_INTERFACE_FN(rmw_init_options_copy,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(const rmw_init_options_t *, rmw_init_options_t *))

RMW_INTERFACE_FN(rmw_init_options_fini,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(rmw_init_options_t *))

RMW_INTERFACE_FN(rmw_shutdown,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(rmw_context_t *))

RMW_INTERFACE_FN(rmw_context_fini,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(rmw_context_t *))

RMW_INTERFACE_FN(rmw_get_serialization_format,
  const char *, nullptr,
  0, ARG_TYPES(void))

RMW_INTERFACE_FN(rmw_create_node,
  rmw_node_t *, nullptr,
  6, ARG_TYPES(
    rmw_context_t *, const char *, const char *, size_t, const rmw_node_security_options_t *,
    bool))

RMW_INTERFACE_FN(rmw_destroy_node,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(rmw_node_t *))

RMW_INTERFACE_FN(rmw_node_assert_liveliness,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(const rmw_node_t *))

RMW_INTERFACE_FN(rmw_node_get_graph_guard_condition,
  const rmw_guard_condition_t *, nullptr,
  1, ARG_TYPES(const rmw_node_t *))

RMW_INTERFACE_FN(rmw_init_publisher_allocation,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(
    const rosidl_message_type_support_t *,
    const rosidl_message_bounds_t *,
    rmw_publisher_allocation_t *))

RMW_INTERFACE_FN(rmw_fini_publisher_allocation,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(rmw_publisher_allocation_t *))

RMW_INTERFACE_FN(rmw_create_publisher,
  rmw_publisher_t *, nullptr,
  5, ARG_TYPES(
    const rmw_node_t *, const rosidl_message_type_support_t *, const char *,
    const rmw_qos_profile_t *, const rmw_publisher_options_t *))

RMW_INTERFACE_FN(rmw_destroy_publisher,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(rmw_node_t *, rmw_publisher_t *))

RMW_INTERFACE_FN(rmw_borrow_loaned_message,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(
    const rmw_publisher_t *,
    const rosidl_message_type_support_t *,
    void **))

RMW_INTERFACE_FN(rmw_return_loaned_message_from_publisher,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(const rmw_publisher_t *, void *))

RMW_INTERFACE_FN(rmw_publish,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const rmw_publisher_t *, const void *, rmw_publisher_allocation_t *))

RMW_INTERFACE_FN(rmw_publish_loaned_message,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const rmw_publisher_t *, void *, rmw_publisher_allocation_t *))

RMW_INTERFACE_FN(rmw_publisher_count_matched_subscriptions,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(const rmw_publisher_t *, size_t *))

RMW_INTERFACE_FN(rmw_publisher_get_actual_qos,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(const rmw_publisher_t *, rmw_qos_profile_t *))

RMW_INTERFACE_FN(rmw_publish_serialized_message,
  rmw_ret_t, RMW_RET_ERROR,
  3,
  ARG_TYPES(const rmw_publisher_t *, const rmw_serialized_message_t *,
  rmw_publisher_allocation_t *))

RMW_INTERFACE_FN(rmw_get_serialized_message_size,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(
    const rosidl_message_type_support_t *,
    const rosidl_message_bounds_t *,
    size_t *))

RMW_INTERFACE_FN(rmw_publisher_assert_liveliness,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(const rmw_publisher_t *))

RMW_INTERFACE_FN(rmw_serialize,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const void *, const rosidl_message_type_support_t *, rmw_serialized_message_t *))

RMW_INTERFACE_FN(rmw_deserialize,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const rmw_serialized_message_t *, const rosidl_message_type_support_t *, void *))

RMW_INTERFACE_FN(rmw_init_subscription_allocation,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(
    const rosidl_message_type_support_t *,
    const rosidl_message_bounds_t *,
    rmw_subscription_allocation_t *))

RMW_INTERFACE_FN(rmw_fini_subscription_allocation,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(rmw_subscription_allocation_t *))

RMW_INTERFACE_FN(rmw_create_subscription,
  rmw_subscription_t *, nullptr,
  5, ARG_TYPES(
    const rmw_node_t *, const rosidl_message_type_support_t *, const char *,
    const rmw_qos_profile_t *, const rmw_subscription_options_t *))

RMW_INTERFACE_FN(rmw_destroy_subscription,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(rmw_node_t *, rmw_subscription_t *))

RMW_INTERFACE_FN(rmw_subscription_count_matched_publishers,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(const rmw_subscription_t *, size_t *))

RMW_INTERFACE_FN(rmw_subscription_get_actual_qos,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(const rmw_subscription_t *, rmw_qos_profile_t *))

RMW_INTERFACE_FN(rmw_take,
  rmw_ret_t, RMW_RET_ERROR,
  4, ARG_TYPES(const rmw_subscription_t *, void *, bool *, rmw_subscription_allocation_t *))

RMW_INTERFACE_FN(rmw_take_with_info,
  rmw_ret_t, RMW_RET_ERROR,
  5,
  ARG_TYPES(const rmw_subscription_t *, void *, bool *, rmw_message_info_t *,
  rmw_subscription_allocation_t *))

RMW_INTERFACE_FN(rmw_take_serialized_message,
  rmw_ret_t, RMW_RET_ERROR,
  4,
  ARG_TYPES(const rmw_subscription_t *, rmw_serialized_message_t *, bool *,
  rmw_subscription_allocation_t *))

RMW_INTERFACE_FN(rmw_take_serialized_message_with_info,
  rmw_ret_t, RMW_RET_ERROR,
  5, ARG_TYPES(
    const rmw_subscription_t *, rmw_serialized_message_t *, bool *, rmw_message_info_t *,
    rmw_subscription_allocation_t *))

RMW_INTERFACE_FN(rmw_take_loaned_message,
  rmw_ret_t, RMW_RET_ERROR,
  4, ARG_TYPES(
    const rmw_subscription_t *, void **, bool *, rmw_subscription_allocation_t *))

RMW_INTERFACE_FN(rmw_take_loaned_message_with_info,
  rmw_ret_t, RMW_RET_ERROR,
  5, ARG_TYPES(
    const rmw_subscription_t *, void **, bool *, rmw_message_info_t *,
    rmw_subscription_allocation_t *))

RMW_INTERFACE_FN(rmw_return_loaned_message_from_subscription,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(const rmw_subscription_t *, void *))

RMW_INTERFACE_FN(rmw_create_client,
  rmw_client_t *, nullptr,
  4, ARG_TYPES(
    const rmw_node_t *, const rosidl_service_type_support_t *, const char *,
    const rmw_qos_profile_t *))

RMW_INTERFACE_FN(rmw_destroy_client,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(rmw_node_t *, rmw_client_t *))

RMW_INTERFACE_FN(rmw_send_request,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const rmw_client_t *, const void *, int64_t *))

RMW_INTERFACE_FN(rmw_take_response,
  rmw_ret_t, RMW_RET_ERROR,
  4, ARG_TYPES(const rmw_client_t *, rmw_request_id_t *, void *, bool *))

RMW_INTERFACE_FN(rmw_create_service,
  rmw_service_t *, nullptr,
  4, ARG_TYPES(
    const rmw_node_t *, const rosidl_service_type_support_t *, const char *,
    const rmw_qos_profile_t *))

RMW_INTERFACE_FN(rmw_destroy_service,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(rmw_node_t *, rmw_service_t *))

RMW_INTERFACE_FN(rmw_take_request,
  rmw_ret_t, RMW_RET_ERROR,
  4, ARG_TYPES(const rmw_service_t *, rmw_request_id_t *, void *, bool *))

RMW_INTERFACE_FN(rmw_send_response,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const rmw_service_t *, rmw_request_id_t *, void *))

RMW_INTERFACE_FN(rmw_take_event,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const rmw_event_t *, void *, bool *))

RMW_INTERFACE_FN(rmw_create_guard_condition,
  rmw_guard_condition_t *, nullptr,
  1, ARG_TYPES(rmw_context_t *))

RMW_INTERFACE_FN(rmw_destroy_guard_condition,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(rmw_guard_condition_t *))

RMW_INTERFACE_FN(rmw_trigger_guard_condition,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(const rmw_guard_condition_t *))

RMW_INTERFACE_FN(rmw_create_wait_set,
  rmw_wait_set_t *, nullptr,
  2, ARG_TYPES(rmw_context_t *, size_t))

RMW_INTERFACE_FN(rmw_destroy_wait_set,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(rmw_wait_set_t *))

RMW_INTERFACE_FN(rmw_wait,
  rmw_ret_t, RMW_RET_ERROR,
  7, ARG_TYPES(
    rmw_subscriptions_t *, rmw_guard_conditions_t *, rmw_services_t *, rmw_clients_t *,
    rmw_events_t *, rmw_wait_set_t *, const rmw_time_t *))

RMW_INTERFACE_FN(rmw_get_publisher_names_and_types_by_node,
  rmw_ret_t, RMW_RET_ERROR,
  6, ARG_TYPES(
    const rmw_node_t *, rcutils_allocator_t *, const char *, const char *, bool,
    rmw_names_and_types_t *))

RMW_INTERFACE_FN(rmw_get_subscriber_names_and_types_by_node,
  rmw_ret_t, RMW_RET_ERROR,
  6, ARG_TYPES(
    const rmw_node_t *, rcutils_allocator_t *, const char *, const char *, bool,
    rmw_names_and_types_t *))

RMW_INTERFACE_FN(rmw_get_service_names_and_types_by_node,
  rmw_ret_t, RMW_RET_ERROR,
  5, ARG_TYPES(
    const rmw_node_t *, rcutils_allocator_t *, const char *, const char *,
    rmw_names_and_types_t *))

RMW_INTERFACE_FN(rmw_get_client_names_and_types_by_node,
  rmw_ret_t, RMW_RET_ERROR,
  5, ARG_TYPES(
    const rmw_node_t *, rcutils_allocator_t *, const char *, const char *,
    rmw_names_and_types_t *))

RMW_INTERFACE_FN(rmw_get_topic_names_and_types,
  rmw_ret_t, RMW_RET_ERROR,
  4, ARG_TYPES(
    const rmw_node_t *, rcutils_allocator_t *, bool,
    rmw_names_and_types_t *))

RMW_INTERFACE_FN(rmw_get_service_names_and_types,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(
    const rmw_node_t *, rcutils_allocator_t *,
    rmw_names_and_types_t *))

RMW_INTERFACE_FN(rmw_get_node_names,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const rmw_node_t *, rcutils_string_array_t *, rcutils_string_array_t *))

RMW_INTERFACE_FN(rmw_count_publishers,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const rmw_node_t *, const char *, size_t *))

RMW_INTERFACE_FN(rmw_count_subscribers,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const rmw_node_t *, const char *, size_t *))

RMW_INTERFACE_FN(rmw_get_gid_for_publisher,
  rmw_ret_t, RMW_RET_ERROR,
  2, ARG_TYPES(const rmw_publisher_t *, rmw_gid_t *))

RMW_INTERFACE_FN(rmw_compare_gids_equal,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const rmw_gid_t *, const rmw_gid_t *, bool *))

RMW_INTERFACE_FN(rmw_service_server_is_available,
  rmw_ret_t, RMW_RET_ERROR,
  3, ARG_TYPES(const rmw_node_t *, const rmw_client_t *, bool *))

RMW_INTERFACE_FN(rmw_set_log_severity,
  rmw_ret_t, RMW_RET_ERROR,
  1, ARG_TYPES(rmw_log_severity_t))

#define GET_SYMBOL(x) symbol_ ## x = get_symbol(#x);

void prefetch_symbols(void)
{
  // get all symbols to avoid race conditions later since the passed
  // symbol name is expected to be a std::string which requires allocation
  GET_SYMBOL(rmw_get_implementation_identifier)
  GET_SYMBOL(rmw_init_options_init)
  GET_SYMBOL(rmw_init_options_copy)
  GET_SYMBOL(rmw_init_options_fini)
  GET_SYMBOL(rmw_shutdown)
  GET_SYMBOL(rmw_context_fini)
  GET_SYMBOL(rmw_get_serialization_format)
  GET_SYMBOL(rmw_create_node)
  GET_SYMBOL(rmw_destroy_node)
  GET_SYMBOL(rmw_node_assert_liveliness)
  GET_SYMBOL(rmw_node_get_graph_guard_condition)
  GET_SYMBOL(rmw_init_publisher_allocation);
  GET_SYMBOL(rmw_fini_publisher_allocation);
  GET_SYMBOL(rmw_create_publisher)
  GET_SYMBOL(rmw_destroy_publisher)
  GET_SYMBOL(rmw_borrow_loaned_message);
  GET_SYMBOL(rmw_return_loaned_message_from_publisher);
  GET_SYMBOL(rmw_publish)
  GET_SYMBOL(rmw_publish_loaned_message)
  GET_SYMBOL(rmw_publisher_count_matched_subscriptions)
  GET_SYMBOL(rmw_publisher_get_actual_qos);
  GET_SYMBOL(rmw_publish_serialized_message)
  GET_SYMBOL(rmw_publisher_assert_liveliness)
  GET_SYMBOL(rmw_get_serialized_message_size)
  GET_SYMBOL(rmw_serialize)
  GET_SYMBOL(rmw_deserialize)
  GET_SYMBOL(rmw_init_subscription_allocation)
  GET_SYMBOL(rmw_fini_subscription_allocation)
  GET_SYMBOL(rmw_create_subscription)
  GET_SYMBOL(rmw_destroy_subscription)
  GET_SYMBOL(rmw_subscription_count_matched_publishers);
  GET_SYMBOL(rmw_subscription_get_actual_qos);
  GET_SYMBOL(rmw_take)
  GET_SYMBOL(rmw_take_with_info)
  GET_SYMBOL(rmw_take_serialized_message)
  GET_SYMBOL(rmw_take_serialized_message_with_info)
  GET_SYMBOL(rmw_take_loaned_message)
  GET_SYMBOL(rmw_take_loaned_message_with_info)
  GET_SYMBOL(rmw_return_loaned_message_from_subscription)
  GET_SYMBOL(rmw_create_client)
  GET_SYMBOL(rmw_destroy_client)
  GET_SYMBOL(rmw_send_request)
  GET_SYMBOL(rmw_take_response)
  GET_SYMBOL(rmw_create_service)
  GET_SYMBOL(rmw_destroy_service)
  GET_SYMBOL(rmw_take_request)
  GET_SYMBOL(rmw_send_response)
  GET_SYMBOL(rmw_take_event)
  GET_SYMBOL(rmw_create_guard_condition)
  GET_SYMBOL(rmw_destroy_guard_condition)
  GET_SYMBOL(rmw_trigger_guard_condition)
  GET_SYMBOL(rmw_create_wait_set)
  GET_SYMBOL(rmw_destroy_wait_set)
  GET_SYMBOL(rmw_wait)
  GET_SYMBOL(rmw_get_publisher_names_and_types_by_node)
  GET_SYMBOL(rmw_get_subscriber_names_and_types_by_node)
  GET_SYMBOL(rmw_get_service_names_and_types_by_node)
  GET_SYMBOL(rmw_get_client_names_and_types_by_node)
  GET_SYMBOL(rmw_get_topic_names_and_types)
  GET_SYMBOL(rmw_get_service_names_and_types)
  GET_SYMBOL(rmw_get_node_names)
  GET_SYMBOL(rmw_count_publishers)
  GET_SYMBOL(rmw_count_subscribers)
  GET_SYMBOL(rmw_get_gid_for_publisher)
  GET_SYMBOL(rmw_compare_gids_equal)
  GET_SYMBOL(rmw_service_server_is_available)
  GET_SYMBOL(rmw_set_log_severity)
}

void * symbol_rmw_init = nullptr;

rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  prefetch_symbols();
  if (!symbol_rmw_init) {
    symbol_rmw_init = get_symbol("rmw_init");
  }
  if (!symbol_rmw_init) {
    return RMW_RET_ERROR;
  }

  typedef rmw_ret_t (* FunctionSignature)(const rmw_init_options_t *, rmw_context_t *);
  FunctionSignature func = reinterpret_cast<FunctionSignature>(symbol_rmw_init);
  return func(options, context);
}

#ifdef __cplusplus
}
#endif
