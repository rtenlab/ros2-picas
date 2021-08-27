// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <map>
#include <set>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rmw/types.h"
#include "rmw_connext_shared_cpp/topic_cache.hpp"
#include "rmw_connext_shared_cpp/types.hpp"

bool operator!=(const rmw_qos_profile_t & lhs, const rmw_qos_profile_t & rhs)
{
  if (lhs.durability != rhs.durability) {
    return true;
  }
  if (lhs.reliability != rhs.reliability) {
    return true;
  }
  if (lhs.liveliness != rhs.liveliness) {
    return true;
  }
  if (lhs.liveliness_lease_duration.sec != rhs.liveliness_lease_duration.sec) {
    return true;
  }
  if (lhs.liveliness_lease_duration.nsec != rhs.liveliness_lease_duration.nsec) {
    return true;
  }
  if (lhs.deadline.sec != rhs.deadline.sec) {
    return true;
  }
  if (lhs.deadline.nsec != rhs.deadline.nsec) {
    return true;
  }
  if (lhs.lifespan.sec != rhs.lifespan.sec) {
    return true;
  }
  if (lhs.lifespan.nsec != rhs.lifespan.nsec) {
    return true;
  }

  return false;
}

bool operator==(const DDSTopicEndpointInfo & lhs, const DDSTopicEndpointInfo & rhs)
{
  // TOPIC
  if (lhs.topic_name != rhs.topic_name) {
    return false;
  }
  if (lhs.topic_type != rhs.topic_type) {
    return false;
  }

  // PARTICIPANT GUID
  if (lhs.participant_guid != rhs.participant_guid) {
    return false;
  }

  // GUID
  if (lhs.endpoint_guid != rhs.endpoint_guid) {
    return false;
  }

  // QOS
  if (lhs.qos_profile != rhs.qos_profile) {
    return false;
  }

  return true;
}

class TopicCacheTestFixture : public ::testing::Test
{
public:
  CustomDataReaderListener topic_cache{nullptr, nullptr};
  DDS::GUID_t participant_guid[2];
  DDS::GUID_t guid[4];
  rmw_qos_profile_t rmw_qos[2];

  void SetUp()
  {
    int guid_count = 1;

    // Instantiate participant GUIDs
    for (int i = 0; i < 2; ++i) {
      memset(&participant_guid[i], guid_count, sizeof(DDS::GUID_t));
      ++guid_count;
    }

    // Instantiate endpoint GUIDs
    for (int i = 0; i < 4; ++i) {
      memset(&guid[i], guid_count, sizeof(DDS::GUID_t));
      ++guid_count;
    }

    // rmw qos
    rmw_qos[0].durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    rmw_qos[0].reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos[0].liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    rmw_qos[0].liveliness_lease_duration.sec = 80u;
    rmw_qos[0].liveliness_lease_duration.nsec = 5555555u;
    rmw_qos[0].deadline.sec = 123u;
    rmw_qos[0].deadline.nsec = 5678u;
    rmw_qos[0].lifespan.sec = 190u;
    rmw_qos[0].lifespan.nsec = 1234u;

    // rmw qos
    rmw_qos[1].durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    rmw_qos[1].reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_qos[1].liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    rmw_qos[1].liveliness_lease_duration.sec = 8u;
    rmw_qos[1].liveliness_lease_duration.nsec = 78901234u;
    rmw_qos[1].deadline.sec = 12u;
    rmw_qos[1].deadline.nsec = 1234u;
    rmw_qos[1].lifespan.sec = 19u;
    rmw_qos[1].lifespan.nsec = 5432u;

    // Add data to topic_cache
    topic_cache.add_information(
      participant_guid[0], guid[0], "topic1", "type1", rmw_qos[0], Publisher);
    topic_cache.add_information(
      participant_guid[0], guid[1], "topic2", "type2", rmw_qos[0], Subscriber);
    topic_cache.add_information(
      participant_guid[1], guid[2], "topic1", "type1", rmw_qos[1], Subscriber);
    topic_cache.add_information(
      participant_guid[1], guid[3], "topic2", "type2", rmw_qos[1], Publisher);
  }
};

TEST_F(TopicCacheTestFixture, test_topic_cache_get_topic_types)
{
  std::map<std::string, std::set<std::string>> topic_type_map;
  topic_cache.fill_topic_names_and_types(true, topic_type_map);

  // there should be only two topics found
  EXPECT_EQ(2u, topic_type_map.size());

  // topic1
  const auto & it = topic_type_map.find("topic1");
  ASSERT_NE(topic_type_map.end(), it);
  // Verify that the object returned from the map is indeed the one expected
  const auto & topic1_types = it->second;
  // Verify that topic1 only has type1
  EXPECT_EQ(1u, topic1_types.size());
  EXPECT_EQ(1u, topic1_types.count("type1"));

  // topic2
  const auto & it2 = topic_type_map.find("topic2");
  ASSERT_NE(topic_type_map.end(), it2);
  // Verify that the object returned from the map is indeed the one expected
  const auto & topic2_types = it2->second;
  // Verify that topic2 only has type2
  EXPECT_EQ(1u, topic2_types.size());
  EXPECT_EQ(1u, topic2_types.count("type2"));
}

TEST_F(TopicCacheTestFixture, test_topic_cache_get_participant_map)
{
  // participant 1
  std::map<std::string, std::set<std::string>> topic_type_map;
  topic_cache.fill_topic_names_and_types_by_guid(true, topic_type_map, participant_guid[0]);
  ASSERT_EQ(2u, topic_type_map.size());
  // Verify that topic1 and respective types are present
  auto topic1it = topic_type_map.find("topic1");
  ASSERT_NE(topic_type_map.end(), topic1it);
  const auto & topic1_types = topic1it->second;
  EXPECT_EQ(1u, topic1_types.count("type1"));
  // Verify that topic2 and respective types are present
  auto topic2it = topic_type_map.find("topic2");
  ASSERT_NE(topic_type_map.end(), topic2it);
  const auto & topic2_types = topic2it->second;
  EXPECT_EQ(1u, topic2_types.count("type2"));

  // participant 2
  std::map<std::string, std::set<std::string>> topic_type_map2;
  topic_cache.fill_topic_names_and_types_by_guid(true, topic_type_map2, participant_guid[1]);
  ASSERT_EQ(2u, topic_type_map2.size());
  // Verify that topic1 and respective types are present
  auto topic1it2 = topic_type_map2.find("topic1");
  ASSERT_NE(topic_type_map2.end(), topic1it2);
  const auto & topic1_types2 = topic1it2->second;
  EXPECT_EQ(1u, topic1_types2.count("type1"));
  // Verify that topic2 and respective types are present
  auto topic2it2 = topic_type_map2.find("topic2");
  ASSERT_NE(topic_type_map2.end(), topic2it2);
  const auto & topic2_types2 = topic2it2->second;
  EXPECT_EQ(1u, topic2_types2.count("type2"));
}

TEST_F(TopicCacheTestFixture, test_topic_cache_get_topic_name_topic_data_map)
{
  std::map<std::string, std::vector<DDSTopicEndpointInfo>> expected_results;
  expected_results["topic1"].push_back(
    {"topic1", "type1", participant_guid[0], guid[0], rmw_qos[0]});
  expected_results["topic2"].push_back(
    {"topic2", "type2", participant_guid[0], guid[1], rmw_qos[0]});
  expected_results["topic1"].push_back(
    {"topic1", "type1", participant_guid[1], guid[2], rmw_qos[1]});
  expected_results["topic2"].push_back(
    {"topic2", "type2", participant_guid[1], guid[3], rmw_qos[1]});

  std::vector<const DDSTopicEndpointInfo *> topic_data;
  for (const auto & result_it : expected_results) {
    topic_data.clear();
    const auto & topic_name = result_it.first;
    const auto & expected_topic_data = result_it.second;

    topic_cache.fill_topic_endpoint_infos(topic_name, true, topic_data);
    ASSERT_FALSE(topic_data.empty());
    // Verify that the topic has all the associated data
    for (const auto & expected : expected_topic_data) {
      bool found_match = false;
      for (const auto actual : topic_data) {
        if (*actual == expected) {
          found_match = true;
          break;
        }
      }
      EXPECT_TRUE(found_match);
    }
  }
}

TEST_F(TopicCacheTestFixture, test_topic_cache_add_topic)
{
  DDS::GUID_t test_guid;
  memset(&test_guid, 100, sizeof(DDS::GUID_t));

  // Add Topic
  const bool did_add = topic_cache.add_information(
    participant_guid[1], test_guid, "TestTopic", "TestType", rmw_qos[1], Publisher);

  // Verify that the returned value was true
  EXPECT_TRUE(did_add);
}

TEST_F(TopicCacheTestFixture, test_topic_cache_remove_topic_element_exists)
{
  // Remove topic1 of 1st participant
  bool did_remove = topic_cache.remove_information(guid[0], Publisher);
  // Assert that the return was true
  ASSERT_TRUE(did_remove);
  // Verify TopicToTypes
  std::map<std::string, std::set<std::string>> topic_type_map;
  topic_cache.fill_topic_names_and_types(true, topic_type_map);
  auto topic_type_it = topic_type_map.find("topic1");
  ASSERT_NE(topic_type_map.end(), topic_type_it);
  EXPECT_EQ(1u, topic_type_it->second.count("type1"));  // type1 remains for participant_guid[1]
  // Verify ParticipantTopicMap
  std::map<std::string, std::set<std::string>> participant_topic_map;
  topic_cache.fill_topic_names_and_types_by_guid(true, participant_topic_map, participant_guid[0]);
  EXPECT_EQ(0u, participant_topic_map.count("topic1"));
  EXPECT_EQ(1u, participant_topic_map.count("topic2"));
  // Verify TopicNameToTopicTypeMap
  std::vector<const DDSTopicEndpointInfo *> topic_data;
  topic_cache.fill_topic_endpoint_infos("topic1", true, topic_data);
  EXPECT_EQ(1u, topic_data.size());

  // Remove topic1 of 2nd participant
  did_remove = topic_cache.remove_information(guid[2], Publisher);
  // Assert that the return was true
  ASSERT_TRUE(did_remove);
  // Verify TopicToTypes
  std::map<std::string, std::set<std::string>> topic_type_map2;
  topic_cache.fill_topic_names_and_types(true, topic_type_map2);
  EXPECT_EQ(0u, topic_type_map2.count("topic1"));
  // Verify ParticipantTopicMap
  std::map<std::string, std::set<std::string>> participant_topic_map2;
  topic_cache.fill_topic_names_and_types_by_guid(true, participant_topic_map2, participant_guid[1]);
  EXPECT_EQ(0u, participant_topic_map2.count("topic1"));
  EXPECT_EQ(1u, participant_topic_map2.count("topic2"));
  // Verify TopicNameToTopicTypeMap
  std::vector<const DDSTopicEndpointInfo *> topic_data2;
  topic_cache.fill_topic_endpoint_infos("topic1", true, topic_data2);
  EXPECT_EQ(0u, topic_data2.size());
}

TEST_F(TopicCacheTestFixture, test_topic_cache_remove_policy_element_does_not_exist)
{
  DDS::GUID_t test_guid;

  // add topic
  memset(&test_guid, 100, sizeof(DDS::GUID_t));
  topic_cache.add_information(
    participant_guid[1], test_guid, "TestTopic", "TestType", rmw_qos[1], Subscriber);

  // Assert that the return was false
  memset(&test_guid, 101, sizeof(DDS::GUID_t));
  bool did_remove = topic_cache.remove_information(test_guid, Subscriber);
  ASSERT_FALSE(did_remove);
}
