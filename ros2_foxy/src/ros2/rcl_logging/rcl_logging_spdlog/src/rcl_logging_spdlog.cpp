// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <rcpputils/filesystem_helper.hpp>
#include <rcutils/allocator.h>
#include <rcutils/filesystem.h>
#include <rcutils/get_env.h>
#include <rcutils/logging.h>
#include <rcutils/process.h>
#include <rcutils/snprintf.h>
#include <rcutils/time.h>
#include <rcutils/strdup.h>
#include <rcutils/format_string.h>

#include <cerrno>
#include <cinttypes>
#include <memory>
#include <mutex>
#include <utility>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

#include "rcl_logging_spdlog/logging_interface.h"

#define RCL_LOGGING_RET_OK    (0)
#define RCL_LOGGING_RET_ERROR (2)

#ifdef __cplusplus
extern "C" {
#endif

static std::mutex g_logger_mutex;
static std::unique_ptr<spdlog::logger> g_root_logger = nullptr;

static spdlog::level::level_enum map_external_log_level_to_library_level(int external_level)
{
  spdlog::level::level_enum level = spdlog::level::level_enum::off;

  // map to the next highest level of severity
  if (external_level <= RCUTILS_LOG_SEVERITY_DEBUG) {
    level = spdlog::level::level_enum::debug;
  } else if (external_level <= RCUTILS_LOG_SEVERITY_INFO) {
    level = spdlog::level::level_enum::info;
  } else if (external_level <= RCUTILS_LOG_SEVERITY_WARN) {
    level = spdlog::level::level_enum::warn;
  } else if (external_level <= RCUTILS_LOG_SEVERITY_ERROR) {
    level = spdlog::level::level_enum::err;
  } else if (external_level <= RCUTILS_LOG_SEVERITY_FATAL) {
    level = spdlog::level::level_enum::critical;
  }
  return level;
}

rcl_logging_ret_t rcl_logging_external_initialize(
  const char * config_file,
  rcutils_allocator_t allocator)
{
  std::lock_guard<std::mutex> lk(g_logger_mutex);
  // It is possible for this to get called more than once in a process (some of
  // the tests do this implicitly by calling rclcpp::init more than once).
  // If the logger is already setup, don't do anything.
  if (g_root_logger != nullptr) {
    return RCL_LOGGING_RET_OK;
  }

  bool config_file_provided = (nullptr != config_file) && (config_file[0] != '\0');
  if (config_file_provided) {
    // TODO(clalancette): implement support for an external configuration file.
    RCUTILS_SET_ERROR_MSG(
      "spdlog logging backend doesn't currently support external configuration");
    return RCL_LOGGING_RET_ERROR;
  } else {
    // To be compatible with ROS 1, we construct a default filename of
    // the form ~/.ros/log/<exe>_<pid>_<milliseconds-since-epoch>.log

    char * logdir = nullptr;
    rcl_logging_ret_t dir_ret = rcl_logging_get_logging_directory(allocator, &logdir);
    if (RCL_LOGGING_RET_OK != dir_ret) {
      // We couldn't get the log directory, so get out of here without setting up
      // logging.
      RCUTILS_SET_ERROR_MSG("Failed to get logging directory");
      return RCL_LOGGING_RET_ERROR;
    }

    // SPDLOG doesn't automatically create the log directories, so create them
    rcpputils::fs::path logdir_path(logdir);
    if (!rcpputils::fs::create_directories(logdir_path)) {
      RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING("Failed to create log directory: %s", logdir);
      allocator.deallocate(logdir, allocator.state);
      return RCL_LOGGING_RET_ERROR;
    }

    // Now get the milliseconds since the epoch in the local timezone.
    rcutils_time_point_value_t now;
    rcutils_ret_t ret = rcutils_system_time_now(&now);
    if (ret != RCUTILS_RET_OK) {
      // We couldn't get the system time, so get out of here without setting up
      // logging.  We don't need to call RCUTILS_SET_ERROR_MSG either since
      // rcutils_system_time_now() already did it.
      return RCL_LOGGING_RET_ERROR;
    }
    int64_t ms_since_epoch = RCUTILS_NS_TO_MS(now);

    // Get the program name.
    char * basec = rcutils_get_executable_name(allocator);
    if (basec == nullptr) {
      // We couldn't get the program name, so get out of here without setting up
      // logging.
      RCUTILS_SET_ERROR_MSG("Failed to get the executable name");
      return RCL_LOGGING_RET_ERROR;
    }

    char name_buffer[4096] = {0};
    int print_ret = rcutils_snprintf(
      name_buffer, sizeof(name_buffer),
      "%s/%s_%i_%" PRId64 ".log", logdir,
      basec, rcutils_get_pid(), ms_since_epoch);
    allocator.deallocate(logdir, allocator.state);
    allocator.deallocate(basec, allocator.state);
    if (print_ret < 0) {
      RCUTILS_SET_ERROR_MSG("Failed to create log file name string");
      return RCL_LOGGING_RET_ERROR;
    }

    auto sink = std::make_unique<spdlog::sinks::basic_file_sink_mt>(name_buffer, false);
    g_root_logger = std::make_unique<spdlog::logger>("root", std::move(sink));

    g_root_logger->set_pattern("%v");
  }

  return RCL_LOGGING_RET_OK;
}

rcl_logging_ret_t rcl_logging_external_shutdown()
{
  g_root_logger = nullptr;
  return RCL_LOGGING_RET_OK;
}

void rcl_logging_external_log(int severity, const char * name, const char * msg)
{
  (void)name;
  g_root_logger->log(map_external_log_level_to_library_level(severity), msg);
}

rcl_logging_ret_t rcl_logging_external_set_logger_level(const char * name, int level)
{
  (void)name;

  g_root_logger->set_level(map_external_log_level_to_library_level(level));

  return RCL_LOGGING_RET_OK;
}


static char *
rcl_expand_user(const char * path, rcutils_allocator_t allocator)
{
  if (NULL == path) {
    return NULL;
  }

  if ('~' != path[0]) {
    return rcutils_strdup(path, allocator);
  }

  const char * homedir = rcutils_get_home_dir();
  if (NULL == homedir) {
    return NULL;
  }
  return rcutils_format_string_limit(
    allocator,
    strlen(homedir) + strlen(path),
    "%s%s",
    homedir,
    path + 1);
}

rcl_logging_ret_t
rcl_logging_get_logging_directory(rcutils_allocator_t allocator, char ** directory)
{
  if (NULL == directory) {
    RCUTILS_SET_ERROR_MSG("directory argument must not be null");
    return RCL_LOGGING_RET_ERROR;
  }
  if (NULL != *directory) {
    RCUTILS_SET_ERROR_MSG("directory argument must point to null");
    return RCL_LOGGING_RET_ERROR;
  }

  const char * log_dir_env;
  const char * err = rcutils_get_env("ROS_LOG_DIR", &log_dir_env);
  if (NULL != err) {
    RCUTILS_SET_ERROR_MSG("rcutils_get_env failed");
    return RCL_LOGGING_RET_ERROR;
  }
  if ('\0' != *log_dir_env) {
    *directory = rcutils_strdup(log_dir_env, allocator);
    if (NULL == *directory) {
      RCUTILS_SET_ERROR_MSG("rcutils_strdup failed");
      return RCL_LOGGING_RET_ERROR;
    }
  } else {
    const char * ros_home_dir_env;
    err = rcutils_get_env("ROS_HOME", &ros_home_dir_env);
    if (NULL != err) {
      RCUTILS_SET_ERROR_MSG("rcutils_get_env failed");
      return RCL_LOGGING_RET_ERROR;
    }
    char * ros_home_dir;
    if ('\0' == *ros_home_dir_env) {
      ros_home_dir = rcutils_join_path("~", ".ros", allocator);
      if (NULL == ros_home_dir) {
        RCUTILS_SET_ERROR_MSG("rcutils_join_path failed");
        return RCL_LOGGING_RET_ERROR;
      }
    } else {
      ros_home_dir = rcutils_strdup(ros_home_dir_env, allocator);
      if (NULL == ros_home_dir) {
        RCUTILS_SET_ERROR_MSG("rcutils_strdup failed");
        return RCL_LOGGING_RET_ERROR;
      }
    }
    *directory = rcutils_join_path(ros_home_dir, "log", allocator);
    allocator.deallocate(ros_home_dir, allocator.state);
    if (NULL == *directory) {
      RCUTILS_SET_ERROR_MSG("rcutils_join_path failed");
      return RCL_LOGGING_RET_ERROR;
    }
  }

  char * directory_maybe_not_expanded = *directory;
  *directory = rcl_expand_user(directory_maybe_not_expanded, allocator);
  allocator.deallocate(directory_maybe_not_expanded, allocator.state);
  if (NULL == *directory) {
    RCUTILS_SET_ERROR_MSG("rcutils_expand_user failed");
    return RCL_LOGGING_RET_ERROR;
  }

  char * directory_maybe_not_native = *directory;
  *directory = rcutils_to_native_path(directory_maybe_not_native, allocator);
  allocator.deallocate(directory_maybe_not_native, allocator.state);
  if (NULL == *directory) {
    RCUTILS_SET_ERROR_MSG("rcutils_to_native_path failed");
    return RCL_LOGGING_RET_ERROR;
  }
  return RCL_LOGGING_RET_OK;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
