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

#include <fstream>
#include <string>

#include "rmw/error_handling.h"
#include "rmw_connext_shared_cpp/security_logging.hpp"

#include "gmock/gmock.h"

using ::testing::HasSubstr;

namespace
{
// Environment variable names
const char log_file_variable_name[] = "ROS_SECURITY_LOG_FILE";
const char log_publish_variable_name[] = "ROS_SECURITY_LOG_PUBLISH";
const char log_verbosity_variable_name[] = "ROS_SECURITY_LOG_VERBOSITY";

// Connext-specific property names
const char log_file_property_name[] = "com.rti.serv.secure.logging.log_file";
const char verbosity_property_name[] = "com.rti.serv.secure.logging.log_level";
const char distribute_enable_property_name[] = "com.rti.serv.secure.logging.distribute.enable";

void custom_setenv(const std::string & variable_name, const std::string & value)
{
#ifdef _WIN32
  auto ret = _putenv_s(variable_name.c_str(), value.c_str());
#else
  auto ret = setenv(variable_name.c_str(), value.c_str(), 1);
#endif
  if (ret != 0) {
    ADD_FAILURE() << "Unable to set environment variable: expected 0, got " << ret;
  }
}

const char * lookup_property_value(DDS::PropertyQosPolicy & policy, const char * property_name)
{
  auto property = DDS::PropertyQosPolicyHelper::lookup_property(
    policy,
    property_name);

  if (property == nullptr) {
    return nullptr;
  }

  return property->value;
}

const char * log_file_property(DDS::PropertyQosPolicy & policy)
{
  return lookup_property_value(policy, log_file_property_name);
}

const char * verbosity_property(DDS::PropertyQosPolicy & policy)
{
  return lookup_property_value(policy, verbosity_property_name);
}

const char * logging_distribute_enable_property(DDS::PropertyQosPolicy & policy)
{
  return lookup_property_value(policy, distribute_enable_property_name);
}

class SecurityLoggingTest : public ::testing::Test
{
public:
  void SetUp()
  {
    custom_setenv(log_file_variable_name, "");
    custom_setenv(log_publish_variable_name, "");
    custom_setenv(log_verbosity_variable_name, "");
  }

  void TearDown()
  {
    rmw_reset_error();
  }
};
}  // namespace

TEST_F(SecurityLoggingTest, test_nothing_enabled)
{
  DDS::PropertyQosPolicy policy;
  EXPECT_EQ(apply_security_logging_configuration(policy), RMW_RET_OK);
  EXPECT_FALSE(rmw_error_is_set());

  EXPECT_EQ(log_file_property(policy), nullptr);
  EXPECT_EQ(logging_distribute_enable_property(policy), nullptr);
  EXPECT_EQ(verbosity_property(policy), nullptr);
}

TEST_F(SecurityLoggingTest, test_log_to_file)
{
  custom_setenv(log_file_variable_name, "/test.log");

  DDS::PropertyQosPolicy policy;
  EXPECT_EQ(apply_security_logging_configuration(policy), RMW_RET_OK);
  EXPECT_FALSE(rmw_error_is_set());

  EXPECT_STREQ(log_file_property(policy), "/test.log");
  EXPECT_EQ(logging_distribute_enable_property(policy), nullptr);
  EXPECT_EQ(verbosity_property(policy), nullptr);
}

TEST_F(SecurityLoggingTest, test_log_publish_true)
{
  custom_setenv(log_publish_variable_name, "true");

  DDS::PropertyQosPolicy policy;
  EXPECT_EQ(apply_security_logging_configuration(policy), RMW_RET_OK);
  EXPECT_FALSE(rmw_error_is_set());

  EXPECT_EQ(log_file_property(policy), nullptr);
  EXPECT_STREQ(logging_distribute_enable_property(policy), "true");
  EXPECT_EQ(verbosity_property(policy), nullptr);
}

TEST_F(SecurityLoggingTest, test_log_publish_false)
{
  custom_setenv(log_publish_variable_name, "false");

  DDS::PropertyQosPolicy policy;
  EXPECT_EQ(apply_security_logging_configuration(policy), RMW_RET_OK);
  EXPECT_FALSE(rmw_error_is_set());

  EXPECT_EQ(log_file_property(policy), nullptr);
  EXPECT_STREQ(logging_distribute_enable_property(policy), "false");
  EXPECT_EQ(verbosity_property(policy), nullptr);
}

TEST_F(SecurityLoggingTest, test_log_publish_invalid)
{
  custom_setenv(log_publish_variable_name, "invalid");

  DDS::PropertyQosPolicy policy;
  EXPECT_EQ(apply_security_logging_configuration(policy), RMW_RET_ERROR);
  EXPECT_TRUE(rmw_error_is_set());
  EXPECT_THAT(
    rmw_get_error_string().str, HasSubstr(
      "ROS_SECURITY_LOG_PUBLISH is not valid: 'invalid' is not a supported value (use 'true' or "
      "'false')"));
}

TEST_F(SecurityLoggingTest, test_log_verbosity)
{
  custom_setenv(log_verbosity_variable_name, "FATAL");

  DDS::PropertyQosPolicy policy;
  EXPECT_EQ(apply_security_logging_configuration(policy), RMW_RET_OK);
  EXPECT_FALSE(rmw_error_is_set());

  EXPECT_EQ(log_file_property(policy), nullptr);
  EXPECT_EQ(logging_distribute_enable_property(policy), nullptr);
  EXPECT_STREQ(verbosity_property(policy), "0");
}

TEST_F(SecurityLoggingTest, test_log_verbosity_invalid)
{
  custom_setenv(log_verbosity_variable_name, "INVALID_VERBOSITY");

  DDS::PropertyQosPolicy policy;
  EXPECT_EQ(apply_security_logging_configuration(policy), RMW_RET_ERROR);
  EXPECT_TRUE(rmw_error_is_set());
  EXPECT_THAT(
    rmw_get_error_string().str, HasSubstr(
      "ROS_SECURITY_LOG_VERBOSITY is not valid: INVALID_VERBOSITY is not a supported verbosity "
      "(use FATAL, ERROR, WARN, INFO, or DEBUG)"));
}

TEST_F(SecurityLoggingTest, test_all)
{
  custom_setenv(log_file_variable_name, "/test.log");
  custom_setenv(log_publish_variable_name, "true");
  custom_setenv(log_verbosity_variable_name, "ERROR");

  DDS::PropertyQosPolicy policy;
  EXPECT_EQ(apply_security_logging_configuration(policy), RMW_RET_OK);
  EXPECT_FALSE(rmw_error_is_set());

  EXPECT_STREQ(log_file_property(policy), "/test.log");
  EXPECT_STREQ(logging_distribute_enable_property(policy), "true");
  EXPECT_STREQ(verbosity_property(policy), "3");
}
