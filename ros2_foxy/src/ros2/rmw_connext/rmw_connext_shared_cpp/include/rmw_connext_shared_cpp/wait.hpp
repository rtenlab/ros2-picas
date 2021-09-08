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

#ifndef RMW_CONNEXT_SHARED_CPP__WAIT_HPP_
#define RMW_CONNEXT_SHARED_CPP__WAIT_HPP_

#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "ndds_include.hpp"

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/types.h"

#include "rmw_connext_shared_cpp/condition_error.hpp"
#include "rmw_connext_shared_cpp/event_converter.hpp"
#include "rmw_connext_shared_cpp/types.hpp"
#include "rmw_connext_shared_cpp/visibility_control.h"
#include "rmw_connext_shared_cpp/connext_static_event_info.hpp"

rmw_ret_t
__gather_event_conditions(
  rmw_events_t * events,
  std::unordered_set<DDS::StatusCondition *> & status_conditions)
{
  if (events) {
    std::unordered_map<DDS::StatusCondition *, DDS::StatusMask> status_mask_map;
    // gather all status conditions and masks
    for (size_t i = 0; i < events->event_count; ++i) {
      auto current_event = static_cast<rmw_event_t *>(events->events[i]);
      RMW_CHECK_ARGUMENT_FOR_NULL(current_event->data, RMW_RET_INVALID_ARGUMENT);
      DDS::Entity * dds_entity = static_cast<ConnextCustomEventInfo *>(
        current_event->data)->get_entity();
      if (!dds_entity) {
        RMW_SET_ERROR_MSG("Event handle is null");
        return RMW_RET_ERROR;
      }
      DDS::StatusCondition * status_condition = dds_entity->get_statuscondition();
      if (!status_condition) {
        RMW_SET_ERROR_MSG("status condition handle is null");
        return RMW_RET_ERROR;
      }
      if (is_event_supported(current_event->event_type)) {
        auto map_pair = status_mask_map.insert(
          std::pair<DDS::StatusCondition *, DDS::StatusMask>(
            status_condition, DDS::STATUS_MASK_NONE));
        auto iterator = map_pair.first;
        status_mask_map[status_condition] = get_status_kind_from_rmw(current_event->event_type) |
          (*iterator).second;
      } else {
        RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("event %d not supported", current_event->event_type);
      }
    }
    for (auto & pair : status_mask_map) {
      // set the status condition's mask with the supported type
      pair.first->set_enabled_statuses(pair.second);
      status_conditions.insert(pair.first);
    }
  }
  return RMW_RET_OK;
}

rmw_ret_t __handle_active_event_conditions(rmw_events_t * events)
{
  // enable a status condition for each event
  if (events) {
    for (size_t i = 0; i < events->event_count; ++i) {
      auto current_event = static_cast<rmw_event_t *>(events->events[i]);
      RMW_CHECK_ARGUMENT_FOR_NULL(current_event->data, RMW_RET_INVALID_ARGUMENT);
      DDS::Entity * dds_entity = static_cast<ConnextCustomEventInfo *>(
        current_event->data)->get_entity();
      if (!dds_entity) {
        RMW_SET_ERROR_MSG("Event handle is null");
        return RMW_RET_ERROR;
      }

      DDS::StatusMask status_mask = dds_entity->get_status_changes();
      bool is_active = false;

      if (is_event_supported(current_event->event_type)) {
        is_active = static_cast<bool>(status_mask &
          get_status_kind_from_rmw(current_event->event_type));
      }
      // if status condition is not found in the active set
      // reset the subscriber handle
      if (!is_active) {
        events->events[i] = nullptr;
      }
    }
  }
  return RMW_RET_OK;
}

rmw_ret_t __detach_condition(
  DDS::WaitSet * dds_wait_set,
  DDS::Condition * condition)
{
  DDS::ReturnCode_t dds_return_code = dds_wait_set->detach_condition(condition);
  rmw_ret_t from_dds = check_dds_ret_code(dds_return_code);
  if (from_dds != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("Failed to get detach condition from wait set");
    return from_dds;
  }
  return RMW_RET_OK;
}

template<typename SubscriberInfo, typename ServiceInfo, typename ClientInfo>
rmw_ret_t
wait(
  const char * implementation_identifier,
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  // To ensure that we properly clean up the wait set, we declare an
  // object whose destructor will detach what we attached (this was previously
  // being done inside the destructor of the wait set.
  struct atexit_t
  {
    ~atexit_t()
    {
      // Manually detach conditions and clear sequences, to ensure a clean wait set for next time.
      if (!wait_set) {
        RMW_SET_ERROR_MSG("wait set handle is null");
        return;
      }
      RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
        wait set handle,
        wait_set->implementation_identifier, implementation_identifier,
        return )
      auto wait_set_info = static_cast<ConnextWaitSetInfo *>(wait_set->data);
      if (!wait_set_info) {
        RMW_SET_ERROR_MSG("WaitSet implementation struct is null");
        return;
      }

      DDS::WaitSet * dds_wait_set = wait_set_info->wait_set;
      if (!dds_wait_set) {
        RMW_SET_ERROR_MSG("DDS wait set handle is null");
        return;
      }

      DDS::ConditionSeq * attached_conditions = wait_set_info->attached_conditions;
      if (!attached_conditions) {
        RMW_SET_ERROR_MSG("DDS condition sequence handle is null");
        return;
      }

      DDS::ReturnCode_t retcode;
      retcode = dds_wait_set->get_conditions(*attached_conditions);
      if (retcode != DDS::RETCODE_OK) {
        RMW_SET_ERROR_MSG("Failed to get attached conditions for wait set");
        return;
      }

      for (DDS::Long i = 0; i < attached_conditions->length(); ++i) {
        auto rmw_ret_code = __detach_condition(dds_wait_set, (*attached_conditions)[i]);
        if (rmw_ret_code != RMW_RET_OK) {
          RMW_SET_ERROR_MSG("Failed to get detatch conditions for wait set");
          return;
        }
      }
    }
    rmw_wait_set_t * wait_set = nullptr;
    const char * implementation_identifier = nullptr;
  } atexit;

  atexit.wait_set = wait_set;
  atexit.implementation_identifier = implementation_identifier;

  if (!wait_set) {
    RMW_SET_ERROR_MSG("wait set handle is null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    wait set handle,
    wait_set->implementation_identifier, implementation_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto wait_set_info = static_cast<ConnextWaitSetInfo *>(wait_set->data);
  if (!wait_set_info) {
    RMW_SET_ERROR_MSG("WaitSet implementation struct is null");
    return RMW_RET_ERROR;
  }

  DDS::WaitSet * dds_wait_set = wait_set_info->wait_set;
  if (!dds_wait_set) {
    RMW_SET_ERROR_MSG("DDS wait set handle is null");
    return RMW_RET_ERROR;
  }

  DDS::ConditionSeq * active_conditions = wait_set_info->active_conditions;
  if (!active_conditions) {
    RMW_SET_ERROR_MSG("DDS condition sequence handle is null");
    return RMW_RET_ERROR;
  }

  // add a condition for each subscriber
  if (subscriptions) {
    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
      auto subscriber_info =
        static_cast<SubscriberInfo *>(subscriptions->subscribers[i]);
      if (!subscriber_info) {
        RMW_SET_ERROR_MSG("subscriber info handle is null");
        return RMW_RET_ERROR;
      }
      DDS::ReadCondition * read_condition = subscriber_info->read_condition_;
      if (!read_condition) {
        RMW_SET_ERROR_MSG("read condition handle is null");
        return RMW_RET_ERROR;
      }
      rmw_ret_t rmw_status = check_attach_condition_error(
        dds_wait_set->attach_condition(read_condition));
      if (rmw_status != RMW_RET_OK) {
        return rmw_status;
      }
    }
  }

  std::unordered_set<DDS::StatusCondition *> status_conditions;
  // gather all status conditions with set masks
  rmw_ret_t ret_code = __gather_event_conditions(events, status_conditions);
  if (ret_code != RMW_RET_OK) {
    return ret_code;
  }
  // enable a status condition for each event
  for (auto status_condition : status_conditions) {
    rmw_ret_t rmw_status = check_attach_condition_error(
      dds_wait_set->attach_condition(status_condition));
    if (rmw_status != RMW_RET_OK) {
      return rmw_status;
    }
  }

  // add a condition for each guard condition
  if (guard_conditions) {
    for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
      auto guard_condition =
        static_cast<DDS::GuardCondition *>(guard_conditions->guard_conditions[i]);
      if (!guard_condition) {
        RMW_SET_ERROR_MSG("guard condition handle is null");
        return RMW_RET_ERROR;
      }
      rmw_ret_t rmw_status = check_attach_condition_error(
        dds_wait_set->attach_condition(guard_condition));
      if (rmw_status != RMW_RET_OK) {
        return rmw_status;
      }
    }
  }

  // add a condition for each service
  if (services) {
    for (size_t i = 0; i < services->service_count; ++i) {
      auto service_info =
        static_cast<ServiceInfo *>(services->services[i]);

      if (!service_info) {
        RMW_SET_ERROR_MSG("service info handle is null");
        return RMW_RET_ERROR;
      }

      DDS::ReadCondition * read_condition = service_info->read_condition_;
      if (!read_condition) {
        RMW_SET_ERROR_MSG("read condition handle is null");
        return RMW_RET_ERROR;
      }
      rmw_ret_t rmw_status = check_attach_condition_error(
        dds_wait_set->attach_condition(read_condition));
      if (rmw_status != RMW_RET_OK) {
        return rmw_status;
      }
    }
  }

  // add a condition for each client
  if (clients) {
    for (size_t i = 0; i < clients->client_count; ++i) {
      auto client_info =
        static_cast<ClientInfo *>(clients->clients[i]);
      if (!client_info) {
        RMW_SET_ERROR_MSG("client info handle is null");
        return RMW_RET_ERROR;
      }

      DDS::DataReader * response_datareader = client_info->response_datareader_;
      if (!response_datareader) {
        RMW_SET_ERROR_MSG("response datareader handle is null");
        return RMW_RET_ERROR;
      }

      // MIGHT BE IMPORTANT !!!
      DDS::ReadCondition * read_condition = client_info->read_condition_;
      if (!read_condition) {
        RMW_SET_ERROR_MSG("read condition handle is null");
        return RMW_RET_ERROR;
      }
      rmw_ret_t rmw_status = check_attach_condition_error(
        dds_wait_set->attach_condition(read_condition));
      if (rmw_status != RMW_RET_OK) {
        return rmw_status;
      }
    }
  }

  // invoke wait until one of the conditions triggers
  DDS::Duration_t timeout;
  if (!wait_timeout) {
    timeout.sec = DDS::DURATION_INFINITE_SEC;
    timeout.nanosec = DDS::DURATION_INFINITE_NSEC;
  } else {
    timeout.sec = static_cast<DDS::Long>(wait_timeout->sec);
    timeout.nanosec = static_cast<DDS::Long>(wait_timeout->nsec);
  }

  DDS::ReturnCode_t status = dds_wait_set->wait(*active_conditions, timeout);

  if (status != DDS::RETCODE_OK && status != DDS::RETCODE_TIMEOUT) {
    RMW_SET_ERROR_MSG("failed to wait on wait set");
    return RMW_RET_ERROR;
  }

  // set subscriber handles to zero for all not triggered conditions
  if (subscriptions) {
    for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
      auto subscriber_info =
        static_cast<SubscriberInfo *>(subscriptions->subscribers[i]);
      if (!subscriber_info) {
        RMW_SET_ERROR_MSG("subscriber info handle is null");
        return RMW_RET_ERROR;
      }
      DDS::ReadCondition * read_condition = subscriber_info->read_condition_;
      if (!read_condition) {
        RMW_SET_ERROR_MSG("read condition handle is null");
        return RMW_RET_ERROR;
      }

      // search for subscriber condition in active set
      DDS::Long j = 0;
      for (; j < active_conditions->length(); ++j) {
        if ((*active_conditions)[j] == read_condition) {
          break;
        }
      }
      // if subscriber condition is not found in the active set
      // reset the subscriber handle
      if (!(j < active_conditions->length())) {
        subscriptions->subscribers[i] = 0;
      }
      rmw_ret_t rmw_ret_code = __detach_condition(dds_wait_set, read_condition);
      if (rmw_ret_code != RMW_RET_OK) {
        return rmw_ret_code;
      }
    }
  }

  // set guard condition handles to zero for all not triggered conditions
  if (guard_conditions) {
    for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
      auto condition =
        static_cast<DDS::Condition *>(guard_conditions->guard_conditions[i]);
      if (!condition) {
        RMW_SET_ERROR_MSG("condition handle is null");
        return RMW_RET_ERROR;
      }

      // search for guard condition in active set
      DDS::Long j = 0;
      for (; j < active_conditions->length(); ++j) {
        if ((*active_conditions)[j] == condition) {
          auto guard = dynamic_cast<DDS::GuardCondition *>(condition);
          DDS::ReturnCode_t guard_status = guard->set_trigger_value(DDS::BOOLEAN_FALSE);
          if (guard_status != DDS::RETCODE_OK) {
            RMW_SET_ERROR_MSG("failed to set trigger value");
            return RMW_RET_ERROR;
          }
          break;
        }
      }
      // if guard condition is not found in the active set
      // reset the guard handle
      if (!(j < active_conditions->length())) {
        guard_conditions->guard_conditions[i] = nullptr;
      }
      rmw_ret_t rmw_ret_code = __detach_condition(dds_wait_set, condition);
      if (rmw_ret_code != RMW_RET_OK) {
        return rmw_ret_code;
      }
    }
  }

  // set service handles to zero for all not triggered conditions
  if (services) {
    for (size_t i = 0; i < services->service_count; ++i) {
      auto service_info =
        static_cast<ServiceInfo *>(services->services[i]);
      if (!service_info) {
        RMW_SET_ERROR_MSG("service info handle is null");
        return RMW_RET_ERROR;
      }
      DDS::ReadCondition * read_condition = service_info->read_condition_;
      if (!read_condition) {
        RMW_SET_ERROR_MSG("read condition handle is null");
        return RMW_RET_ERROR;
      }

      // search for service condition in active set
      DDS::Long j = 0;
      for (; j < active_conditions->length(); ++j) {
        if ((*active_conditions)[j] == read_condition) {
          break;
        }
      }
      // if service condition is not found in the active set
      // reset the subscriber handle
      if (!(j < active_conditions->length())) {
        services->services[i] = nullptr;
      }
      rmw_ret_t rmw_ret_code = __detach_condition(dds_wait_set, read_condition);
      if (rmw_ret_code != RMW_RET_OK) {
        return rmw_ret_code;
      }
    }
  }

  // set client handles to zero for all not triggered conditions
  if (clients) {
    for (size_t i = 0; i < clients->client_count; ++i) {
      auto client_info =
        static_cast<ClientInfo *>(clients->clients[i]);
      if (!client_info) {
        RMW_SET_ERROR_MSG("client info handle is null");
        return RMW_RET_ERROR;
      }
      DDS::ReadCondition * read_condition = client_info->read_condition_;
      if (!read_condition) {
        RMW_SET_ERROR_MSG("read condition handle is null");
        return RMW_RET_ERROR;
      }

      // search for service condition in active set
      DDS::Long j = 0;
      for (; j < active_conditions->length(); ++j) {
        if ((*active_conditions)[j] == read_condition) {
          break;
        }
      }
      // if client condition is not found in the active set
      // reset the subscriber handle
      if (!(j < active_conditions->length())) {
        clients->clients[i] = nullptr;
      }
      rmw_ret_t rmw_ret_code = __detach_condition(dds_wait_set, read_condition);
      if (rmw_ret_code != RMW_RET_OK) {
        return rmw_ret_code;
      }
    }
  }
  {
    rmw_ret_t rmw_ret_code = __handle_active_event_conditions(events);
    if (rmw_ret_code != RMW_RET_OK) {
      return rmw_ret_code;
    }
  }

  if (status == DDS::RETCODE_TIMEOUT) {
    return RMW_RET_TIMEOUT;
  }
  return RMW_RET_OK;
}

#endif  // RMW_CONNEXT_SHARED_CPP__WAIT_HPP_
