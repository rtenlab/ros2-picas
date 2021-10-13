// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <rcutils/allocator.h>
#include <rcutils/get_env.h>
#include <rcutils/logging.h>
#include <rcutils/process.h>
#include <rcutils/time.h>

#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include <log4cxx/file.h>
#include <log4cxx/fileappender.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/helpers/transcoder.h>

#include <cerrno>
#include <cinttypes>
#include <stdexcept>
#include <string>

/**
 *  Maps the logger name to the log4cxx logger. If the name is null or empty it will map to the
 *  root logger.
 */
static const log4cxx::LoggerPtr get_logger(const char * name)
{
  if (nullptr == name || '\0' == name[0]) {
    return log4cxx::Logger::getRootLogger();
  }
  return log4cxx::Logger::getLogger(name);
}

static const log4cxx::LevelPtr map_external_log_level_to_library_level(int external_level)
{
  log4cxx::LevelPtr level;
  // map to the next highest level of severity
  if (external_level <= RCUTILS_LOG_SEVERITY_DEBUG) {
    level = log4cxx::Level::getDebug();
  } else if (external_level <= RCUTILS_LOG_SEVERITY_INFO) {
    level = log4cxx::Level::getInfo();
  } else if (external_level <= RCUTILS_LOG_SEVERITY_WARN) {
    level = log4cxx::Level::getWarn();
  } else if (external_level <= RCUTILS_LOG_SEVERITY_ERROR) {
    level = log4cxx::Level::getError();
  } else if (external_level <= RCUTILS_LOG_SEVERITY_FATAL) {
    level = log4cxx::Level::getFatal();
  }
  return level;
}

#ifdef __cplusplus
extern "C" {
#endif

#include "rcl_logging_log4cxx/logging_interface.h"

#define RCL_LOGGING_RET_OK                          (0)
#define RCL_LOGGING_RET_ERROR                       (2)
#define RCL_LOGGING_RET_CONFIG_FILE_DOESNT_EXIST    (21)
#define RCL_LOGGING_RET_CONFIG_FILE_INVALID         (22)

rcl_logging_ret_t rcl_logging_external_initialize(
  const char * config_file,
  rcutils_allocator_t allocator)
{
  (void)allocator;

  bool config_file_provided = (nullptr != config_file) && (config_file[0] != '\0');
  bool use_default_config = !config_file_provided;
  rcl_logging_ret_t status = RCL_LOGGING_RET_OK;

  if (config_file_provided) {
    log4cxx::helpers::Pool pool;
    log4cxx::File file(config_file);
    if (!file.exists(pool)) {
      // The provided config file doesn't exist, fall back to using default configuration
      use_default_config = true;
      status = RCL_LOGGING_RET_CONFIG_FILE_DOESNT_EXIST;
    } else {
      // Attempt to configure the system using the provided config file, but if
      // we fail fall back to using the default configuration
      try {
        log4cxx::PropertyConfigurator::configure(file);
      } catch (std::exception & ex) {
        (void)ex;
        use_default_config = true;
        status = RCL_LOGGING_RET_CONFIG_FILE_INVALID;
      }
    }
  }

  if (use_default_config) {
    // Set the default File Appender on the root logger
    log4cxx::LoggerPtr root_logger(get_logger(nullptr));
    log4cxx::LayoutPtr layout(new log4cxx::PatternLayout(LOG4CXX_STR("%m%n")));

    // To be compatible with ROS 1, we want to construct a default filename of
    // the form ~/.ros/log/<exe>_<pid>_<milliseconds-since-epoch>.log

    // First get the home directory.
    const char * homedir = rcutils_get_home_dir();
    if (homedir == nullptr) {
      // We couldn't get the home directory; it is not really going to be
      // possible to do logging properly, so get out of here without setting
      // up logging.
      return RCL_LOGGING_RET_ERROR;
    }

    // Now get the milliseconds since the epoch in the local timezone.
    rcutils_time_point_value_t now;
    rcutils_ret_t ret = rcutils_system_time_now(&now);
    if (ret != RCUTILS_RET_OK) {
      // We couldn't get the system time, so get out of here without setting up
      // logging.
      return RCL_LOGGING_RET_ERROR;
    }
    int64_t ms_since_epoch = RCUTILS_NS_TO_MS(now);

    // Get the program name.
    char * executable_name = rcutils_get_executable_name(allocator);
    if (executable_name == nullptr) {
      // We couldn't get the program name, so get out of here without setting up
      // logging.
      return RCL_LOGGING_RET_ERROR;
    }

    char log_name_buffer[512] = {0};
    int print_ret = rcutils_snprintf(
      log_name_buffer, sizeof(log_name_buffer),
      "%s/.ros/log/%s_%i_%" PRId64 ".log", homedir, executable_name,
      rcutils_get_pid(), ms_since_epoch);
    allocator.deallocate(executable_name, allocator.state);
    if (print_ret < 0) {
      RCUTILS_SET_ERROR_MSG("Failed to create log file name string");
      return RCL_LOGGING_RET_ERROR;
    }
    std::string log_name_str(log_name_buffer);
    LOG4CXX_DECODE_CHAR(log_name_l4cxx_str, log_name_str);
    log4cxx::FileAppenderPtr file_appender(
      new log4cxx::FileAppender(layout, log_name_l4cxx_str, true));
    root_logger->addAppender(file_appender);
  }

  return status;
}

rcl_logging_ret_t rcl_logging_external_shutdown()
{
  log4cxx::BasicConfigurator::resetConfiguration();
  return RCL_LOGGING_RET_OK;
}

void rcl_logging_external_log(int severity, const char * name, const char * msg)
{
  log4cxx::LoggerPtr logger(get_logger(name));
  log4cxx::LevelPtr level(map_external_log_level_to_library_level(severity));
  logger->log(level, msg);
}

rcl_logging_ret_t rcl_logging_external_set_logger_level(const char * name, int level)
{
  log4cxx::LoggerPtr logger(get_logger(name));
  logger->setLevel(map_external_log_level_to_library_level(level));
  return RCL_LOGGING_RET_OK;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
