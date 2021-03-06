// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

/**
 * @file SubscriberHistory.cpp
 *
 */

#include <fastrtps/subscriber/SubscriberHistory.h>
#include "SubscriberImpl.h"

#include <fastrtps/rtps/reader/RTPSReader.h>
#include "../rtps/reader/WriterProxy.h"

#include <fastrtps/TopicDataType.h>
#include <fastrtps/log/Log.h>

#include <mutex>

namespace eprosima {
namespace fastrtps {

using namespace rtps;

SubscriberHistory::SubscriberHistory(
        SubscriberImpl* simpl,
        uint32_t payloadMaxSize,
        const HistoryQosPolicy& history,
        const ResourceLimitsQosPolicy& resource,
        MemoryManagementPolicy_t mempolicy)
    : ReaderHistory(HistoryAttributes(mempolicy, payloadMaxSize,
                history.kind == KEEP_ALL_HISTORY_QOS ?
                        resource.allocated_samples :
                        simpl->getAttributes().topic.getTopicKind() == NO_KEY ?
                            std::min(resource.allocated_samples, history.depth) :
                            std::min(resource.allocated_samples, history.depth * resource.max_instances),
                history.kind == KEEP_ALL_HISTORY_QOS ?
                        resource.max_samples :
                        simpl->getAttributes().topic.getTopicKind() == NO_KEY ?
                            history.depth :
                            history.depth * resource.max_instances))
    , m_historyQos(history)
    , m_resourceLimitsQos(resource)
    , mp_subImpl(simpl)
    , mp_getKeyObject(nullptr)
{
    if (mp_subImpl->getType()->m_isGetKeyDefined)
    {
        mp_getKeyObject = mp_subImpl->getType()->createData();
    }

    using std::placeholders::_1;
    using std::placeholders::_2;

    if (simpl->getAttributes().topic.getTopicKind() == NO_KEY)
    {
        receive_fn_ = history.kind == KEEP_ALL_HISTORY_QOS ? 
            std::bind(&SubscriberHistory::received_change_keep_all_no_key, this, _1, _2) :
            std::bind(&SubscriberHistory::received_change_keep_last_no_key, this, _1, _2);
    }
    else
    {
        receive_fn_ = history.kind == KEEP_ALL_HISTORY_QOS ? 
            std::bind(&SubscriberHistory::received_change_keep_all_with_key, this, _1, _2) :
            std::bind(&SubscriberHistory::received_change_keep_last_with_key, this, _1, _2);
    }
}

SubscriberHistory::~SubscriberHistory()
{
    if (mp_subImpl->getType()->m_isGetKeyDefined)
    {
        mp_subImpl->getType()->deleteData(mp_getKeyObject);
    }
}

bool SubscriberHistory::received_change(
        CacheChange_t* a_change,
        size_t unknown_missing_changes_up_to)
{
    if (mp_reader == nullptr || mp_mutex == nullptr)
    {
        logError(RTPS_HISTORY, "You need to create a Reader with this History before using it");
        return false;
    }

    std::lock_guard<RecursiveTimedMutex> guard(*mp_mutex);
    return receive_fn_(a_change, unknown_missing_changes_up_to);
}

bool SubscriberHistory::received_change_keep_all_no_key(
        CacheChange_t* a_change,
        size_t unknown_missing_changes_up_to)
{
    // TODO(Ricardo) Check
    if (m_changes.size() + unknown_missing_changes_up_to < static_cast<size_t>(m_resourceLimitsQos.max_samples) )
    {
        return add_received_change(a_change);
    }

    return false;
}

bool SubscriberHistory::received_change_keep_last_no_key(
        CacheChange_t* a_change,
        size_t /* unknown_missing_changes_up_to */ )
{
    bool add = false;
    if (m_changes.size() < static_cast<size_t>(m_historyQos.depth) )
    {
        add = true;
    }
    else
    {
        // Try to substitute the oldest sample.

        // As the history should be ordered following the presentation QoS, we can always remove the first one.
        add = remove_change_sub(m_changes.at(0));
    }

    if (add)
    {
        return add_received_change(a_change);
    }

    return false;
}

bool SubscriberHistory::received_change_keep_all_with_key(
        CacheChange_t* a_change,
        size_t /* unknown_missing_changes_up_to */ )
{
    // TODO(Miguel C): Should we check unknown_missing_changes_up_to as it is done in received_change_keep_all_no_key?

    t_m_Inst_Caches::iterator vit;
    if (find_key_for_change(a_change, vit))
    {
        std::vector<CacheChange_t*>& instance_changes = vit->second.cache_changes;
        if (instance_changes.size() < static_cast<size_t>(m_resourceLimitsQos.max_samples_per_instance) )
        {
            return add_received_change_with_key(a_change, vit->second.cache_changes);
        }

        logWarning(SUBSCRIBER, "Change not added due to maximum number of samples per instance";);
    }

    return false;
}

bool SubscriberHistory::received_change_keep_last_with_key(
        CacheChange_t* a_change,
        size_t /* unknown_missing_changes_up_to */)
{
    t_m_Inst_Caches::iterator vit;
    if (find_key_for_change(a_change, vit))
    {
        bool add = false;
        std::vector<CacheChange_t*>& instance_changes = vit->second.cache_changes;
        if (instance_changes.size() < static_cast<size_t>(m_historyQos.depth) )
        {
            add = true;
        }
        else
        {
            // Try to substitute the oldest sample.

            // As the instance should be ordered following the presentation QoS, we can always remove the first one.
            add = remove_change_sub(instance_changes.at(0));
        }

        if (add)
        {
            return add_received_change_with_key(a_change, instance_changes);
        }
    }

    return false;
}

bool SubscriberHistory::add_received_change(
        CacheChange_t* a_change)
{
    if (m_isHistoryFull)
    {
        // Discarding the sample.
        logWarning(SUBSCRIBER, "Attempting to add Data to Full ReaderHistory: " << mp_subImpl->getGuid().entityId);
        return false;
    }

    if (add_change(a_change))
    {
        if (m_changes.size() == static_cast<size_t>(m_resourceLimitsQos.max_samples) )
        {
            m_isHistoryFull = true;
        }

        logInfo(SUBSCRIBER, mp_subImpl->getGuid().entityId
            << ": Change " << a_change->sequenceNumber << " added from: "
            << a_change->writerGUID;);

        return true;
    }

    return false;
}

bool SubscriberHistory::add_received_change_with_key(
        CacheChange_t* a_change,
        std::vector<CacheChange_t*>& instance_changes)
{
    if (m_isHistoryFull)
    {
        // Discarting the sample.
        logWarning(SUBSCRIBER, "Attempting to add Data to Full ReaderHistory: " << mp_subImpl->getGuid().entityId);
        return false;
    }

    if (add_change(a_change))
    {
        if (m_changes.size() == static_cast<size_t>(m_resourceLimitsQos.max_samples))
        {
            m_isHistoryFull = true;
        }

        //ADD TO KEY VECTOR

        // As the instance should be ordered following the presentation QoS, and
        // we only support ordering by reception timestamp, we can always add at the end.
        instance_changes.push_back(a_change);

        logInfo(SUBSCRIBER, mp_reader->getGuid().entityId
            << ": Change " << a_change->sequenceNumber << " added from: "
            << a_change->writerGUID << " with KEY: " << a_change->instanceHandle;);

        return true;
    }

    return false;
}

bool SubscriberHistory::find_key_for_change(
        rtps::CacheChange_t* a_change,
        t_m_Inst_Caches::iterator& map_it)
{
    if (!a_change->instanceHandle.isDefined() && mp_subImpl->getType() != nullptr)
    {
        logInfo(RTPS_HISTORY, "Getting Key of change with no Key transmitted")
        mp_subImpl->getType()->deserialize(&a_change->serializedPayload, mp_getKeyObject);
        bool is_key_protected = false;
#if HAVE_SECURITY
        is_key_protected = mp_reader->getAttributes().security_attributes().is_key_protected;
#endif
        if (!mp_subImpl->getType()->getKey(mp_getKeyObject, &a_change->instanceHandle, is_key_protected))
        {
            return false;
        }
    }
    else if (!a_change->instanceHandle.isDefined())
    {
        logWarning(RTPS_HISTORY, "NO KEY in topic: " << mp_subImpl->getAttributes().topic.topicName
            << " and no method to obtain it";);
        return false;
    }

    return find_key(a_change, &map_it);
}

void SubscriberHistory::deserialize_change(
        CacheChange_t* change,
        uint32_t ownership_strength,
        void* data,
        SampleInfo_t* info)
{
    if (change->kind == ALIVE)
    {
        mp_subImpl->getType()->deserialize(&change->serializedPayload, data);
    }

    if (info != nullptr)
    {
        info->sampleKind = change->kind;
        info->sample_identity.writer_guid(change->writerGUID);
        info->sample_identity.sequence_number(change->sequenceNumber);
        info->sourceTimestamp = change->sourceTimestamp;
        info->ownershipStrength = ownership_strength;
        if (mp_subImpl->getAttributes().topic.topicKind == WITH_KEY &&
            change->instanceHandle == c_InstanceHandle_Unknown &&
            change->kind == ALIVE)
        {
            bool is_key_protected = false;
#if HAVE_SECURITY
            is_key_protected = mp_reader->getAttributes().security_attributes().is_key_protected;
#endif
            mp_subImpl->getType()->getKey(data, &change->instanceHandle, is_key_protected);
        }
        info->iHandle = change->instanceHandle;
        info->related_sample_identity = change->write_params.sample_identity();
    }
}

bool SubscriberHistory::readNextData(
        void* data,
        SampleInfo_t* info,
        std::chrono::steady_clock::time_point& max_blocking_time)
{
    if (mp_reader == nullptr || mp_mutex == nullptr)
    {
        logError(RTPS_HISTORY, "You need to create a Reader with this History before using it");
        return false;
    }

    std::unique_lock<RecursiveTimedMutex> lock(*mp_mutex, std::defer_lock);

    if(lock.try_lock_until(max_blocking_time))
    {
        CacheChange_t* change;
        WriterProxy * wp;
        if (mp_reader->nextUnreadCache(&change, &wp))
        {
            logInfo(SUBSCRIBER, mp_reader->getGuid().entityId << ": reading " << change->sequenceNumber);
            uint32_t ownership = wp && mp_subImpl->getAttributes().qos.m_ownership.kind == EXCLUSIVE_OWNERSHIP_QOS ?
                wp->ownership_strength() : 0;
            deserialize_change(change, ownership, data, info);
            return true;
        }
    }
    return false;
}


bool SubscriberHistory::takeNextData(
        void* data,
        SampleInfo_t* info,
        std::chrono::steady_clock::time_point& max_blocking_time)
{
    if (mp_reader == nullptr || mp_mutex == nullptr)
    {
        logError(RTPS_HISTORY, "You need to create a Reader with this History before using it");
        return false;
    }

    std::unique_lock<RecursiveTimedMutex> lock(*mp_mutex, std::defer_lock);

    if(lock.try_lock_until(max_blocking_time))
    {
        CacheChange_t* change;
        WriterProxy * wp;
        if (mp_reader->nextUntakenCache(&change, &wp))
        {
            logInfo(SUBSCRIBER, mp_reader->getGuid().entityId << ": taking seqNum" << change->sequenceNumber <<
                    " from writer: " << change->writerGUID);
            uint32_t ownership = wp && mp_subImpl->getAttributes().qos.m_ownership.kind == EXCLUSIVE_OWNERSHIP_QOS ?
                wp->ownership_strength() : 0;
            deserialize_change(change, ownership, data, info);
            remove_change_sub(change);
            return true;
        }
    }

    return false;
}

bool SubscriberHistory::find_key(
        CacheChange_t* a_change,
        t_m_Inst_Caches::iterator* vit_out)
{
    t_m_Inst_Caches::iterator vit;
    vit = keyed_changes_.find(a_change->instanceHandle);
    if (vit != keyed_changes_.end())
    {
        *vit_out = vit;
        return true;
    }

    if (keyed_changes_.size() < static_cast<size_t>(m_resourceLimitsQos.max_instances))
    {
        *vit_out = keyed_changes_.insert(std::make_pair(a_change->instanceHandle, KeyedChanges())).first;
        return true;
    }
    else
    {
        for (vit = keyed_changes_.begin(); vit!= keyed_changes_.end(); ++vit)
        {
            if (vit->second.cache_changes.size() == 0)
            {
                keyed_changes_.erase(vit);
                *vit_out = keyed_changes_.insert(std::make_pair(a_change->instanceHandle, KeyedChanges())).first;
                return true;
            }
        }
        logWarning(SUBSCRIBER, "History has reached the maximum number of instances");
    }
    return false;
}


bool SubscriberHistory::remove_change_sub(
        CacheChange_t* change)
{
    if (mp_reader == nullptr || mp_mutex == nullptr)
    {
        logError(RTPS_HISTORY, "You need to create a Reader with this History before using it");
        return false;
    }

    std::lock_guard<RecursiveTimedMutex> guard(*mp_mutex);
    if (mp_subImpl->getAttributes().topic.getTopicKind() == NO_KEY)
    {
        if (remove_change(change))
        {
            m_isHistoryFull = false;
            return true;
        }
        return false;
    }
    else
    {
        t_m_Inst_Caches::iterator vit;
        if (!find_key(change, &vit))
        {
            return false;
        }

        for (auto chit = vit->second.cache_changes.begin(); chit != vit->second.cache_changes.end(); ++chit)
        {
            if ((*chit)->sequenceNumber == change->sequenceNumber && (*chit)->writerGUID == change->writerGUID)
            {
                if (remove_change(change))
                {
                    vit->second.cache_changes.erase(chit);
                    m_isHistoryFull = false;
                    return true;
                }
            }
        }
        logError(SUBSCRIBER, "Change not found, something is wrong");
    }
    return false;
}

bool SubscriberHistory::set_next_deadline(
        const InstanceHandle_t& handle,
        const std::chrono::steady_clock::time_point& next_deadline_us)
{
    if (mp_reader == nullptr || mp_mutex == nullptr)
    {
        logError(RTPS_HISTORY, "You need to create a Reader with this History before using it");
        return false;
    }
    std::lock_guard<RecursiveTimedMutex> guard(*mp_mutex);

    if (mp_subImpl->getAttributes().topic.getTopicKind() == NO_KEY)
    {
        next_deadline_us_ = next_deadline_us;
        return true;
    }
    else if (mp_subImpl->getAttributes().topic.getTopicKind() == WITH_KEY)
    {
        if (keyed_changes_.find(handle) == keyed_changes_.end())
        {
            return false;
        }

        keyed_changes_[handle].next_deadline_us = next_deadline_us;
        return true;
    }

    return false;
}

bool SubscriberHistory::get_next_deadline(
        InstanceHandle_t &handle,
        std::chrono::steady_clock::time_point &next_deadline_us)
{
    if (mp_reader == nullptr || mp_mutex == nullptr)
    {
        logError(RTPS_HISTORY, "You need to create a Reader with this History before using it");
        return false;
    }
    std::lock_guard<RecursiveTimedMutex> guard(*mp_mutex);

    if (mp_subImpl->getAttributes().topic.getTopicKind() == NO_KEY)
    {
        next_deadline_us = next_deadline_us_;
        return true;
    }
    else if (mp_subImpl->getAttributes().topic.getTopicKind() == WITH_KEY)
    {
        auto min = std::min_element(keyed_changes_.begin(),
                                    keyed_changes_.end(),
                                    [](
                                    const std::pair<InstanceHandle_t, KeyedChanges> &lhs,
                                    const std::pair<InstanceHandle_t, KeyedChanges> &rhs)
        { return lhs.second.next_deadline_us < rhs.second.next_deadline_us; });
        handle = min->first;
        next_deadline_us = min->second.next_deadline_us;
        return true;
    }

    return false;
}

} // namespace fastrtps
} // namsepace eprosima
