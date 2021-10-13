// Copyright 2020 Canonical Ltd
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

#include <stdexcept>
#include <string>

#include "rcpputils/get_env.hpp"
#include "rcutils/logging.h"
#include "rmw/error_handling.h"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"

#include "rmw_connext_shared_cpp/security_logging.hpp"

namespace
{
// Environment variable names
// TODO(security-wg): These are intended to be temporary, and need to be refactored into a proper
// abstraction.
const char log_file_variable_name[] = "ROS_SECURITY_LOG_FILE";
const char log_publish_variable_name[] = "ROS_SECURITY_LOG_PUBLISH";
const char log_verbosity_variable_name[] = "ROS_SECURITY_LOG_VERBOSITY";

// Connext property names. These are being taken from table 10.1 of
// https://community.rti.com/static/documentation/connext-dds/5.3.1/doc/manuals/connext_dds/dds_security/RTI_SecurityPlugins_GettingStarted.pdf
const char log_file_property_name[] = "com.rti.serv.secure.logging.log_file";
const char distribute_enable_property_name[] = "com.rti.serv.secure.logging.distribute.enable";
const char verbosity_property_name[] = "com.rti.serv.secure.logging.log_level";

// Connext 5.3.1 uses numeric levels (although note that v6 moved to using strings). These levels
// are:
// EMERGENCY: 0
// ALERT: 1
// CRITICAL: 2
// ERROR: 3
// WARNING: 4
// NOTICE: 5
// INFORMATIONAL: 6
// DEBUG: 7
//
// ROS has less logging levels, but it makes sense to use them here for consistency, so we have
// the following mapping.
const struct
{
  RCUTILS_LOG_SEVERITY ros_severity;
  const char * const dds_level;
} verbosity_mapping[] =
{
  {RCUTILS_LOG_SEVERITY_FATAL, "0"},
  {RCUTILS_LOG_SEVERITY_ERROR, "3"},
  {RCUTILS_LOG_SEVERITY_WARN, "4"},
  {RCUTILS_LOG_SEVERITY_INFO, "6"},
  {RCUTILS_LOG_SEVERITY_DEBUG, "7"},
};

bool severity_names_str(char buffer[], size_t buffer_size)
{
  size_t offset = 0;

  size_t severity_count = sizeof(verbosity_mapping) / sizeof(verbosity_mapping[0]);
  for (size_t i = 0; i < (severity_count - 1); ++i) {
    RCUTILS_LOG_SEVERITY severity = verbosity_mapping[i].ros_severity;

    int length = rcutils_snprintf(
      buffer + offset, buffer_size - offset, "%s, ",
      g_rcutils_log_severity_names[severity]);

    if (length < 0) {
      return false;
    }

    offset += length;
    if (offset >= buffer_size) {
      return false;
    }
  }

  int length = rcutils_snprintf(
    buffer + offset, buffer_size - offset, "or %s",
    g_rcutils_log_severity_names[verbosity_mapping[severity_count - 1].ros_severity]);

  if (length < 0) {
    return false;
  }

  return (offset + length) <= buffer_size;
}

bool string_to_verbosity_level(const std::string & str, const char ** level)
{
  int ros_severity;
  if (rcutils_logging_severity_level_from_string(
      str.c_str(),
      rcutils_get_default_allocator(), &ros_severity) == RCUTILS_RET_OK)
  {
    for (const auto & item : verbosity_mapping) {
      if (ros_severity == item.ros_severity) {
        *level = item.dds_level;
        return true;
      }
    }
  }

  return false;
}

bool validate_boolean(const std::string & str)
{
  return str == "true" || str == "false";
}

bool apply_property(
  DDS::PropertyQosPolicy & policy, const char * const property_name,
  const std::string & value)
{
  // Overwrite existing properties, so remove it if it already exists
  DDS::PropertyQosPolicyHelper::remove_property(policy, property_name);

  auto status = DDS::PropertyQosPolicyHelper::add_property(
    policy,
    property_name,
    value.c_str(),
    DDS::BOOLEAN_FALSE);

  return DDS::RETCODE_OK == status;
}

bool get_env(const char * variable_name, std::string & variable_value)
{
  try {
    variable_value = rcpputils::get_env_var(variable_name);
  } catch (std::runtime_error & exception) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "unable to get %s environment variable: %s",
      variable_name,
      exception.what());
    return false;
  }

  return true;
}
}  // namespace

rmw_ret_t apply_security_logging_configuration(DDS::PropertyQosPolicy & policy)
{
  std::string env_value;

  // Handle logging to file
  if (!get_env(log_file_variable_name, env_value)) {
    return RMW_RET_ERROR;
  }
  if (!env_value.empty()) {
    if (!apply_property(policy, log_file_property_name, env_value)) {
      RMW_SET_ERROR_MSG("failed to set security logging file");
      return RMW_RET_ERROR;
    }
  }

  // Handle log distribution over DDS
  if (!get_env(log_publish_variable_name, env_value)) {
    return RMW_RET_ERROR;
  }
  if (!env_value.empty()) {
    if (!validate_boolean(env_value)) {
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "%s is not valid: '%s' is not a supported value (use 'true' or 'false')",
        log_publish_variable_name,
        env_value.c_str());
      return RMW_RET_ERROR;
    }

    if (!apply_property(policy, distribute_enable_property_name, env_value)) {
      RMW_SET_ERROR_MSG("failed to set security logging distribute");
      return RMW_RET_ERROR;
    }
  }

  // Handle log verbosity
  if (!get_env(log_verbosity_variable_name, env_value)) {
    return RMW_RET_ERROR;
  }
  if (!env_value.empty()) {
    const char * level;
    if (!string_to_verbosity_level(env_value, &level)) {
      char humanized_severity_list[RCUTILS_ERROR_MESSAGE_MAX_LENGTH];
      if (severity_names_str(humanized_severity_list, RCUTILS_ERROR_MESSAGE_MAX_LENGTH)) {
        RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
          "%s is not valid: %s is not a supported verbosity (use %s)",
          log_verbosity_variable_name,
          env_value.c_str(),
          humanized_severity_list);
      } else {
        RMW_SET_ERROR_MSG("unable to create severity string");
      }

      return RMW_RET_ERROR;
    }

    if (!apply_property(policy, verbosity_property_name, level)) {
      RMW_SET_ERROR_MSG("failed to set security logging verbosity");
      return RMW_RET_ERROR;
    }
  }

  return RMW_RET_OK;
}
