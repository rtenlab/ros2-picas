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
 * @file WriterProxy.cpp
 *
 */

#include <fastrtps/rtps/builtin/data/WriterProxyData.h>

#include "WriterProxy.h"
#include <fastrtps/rtps/reader/StatefulReader.h>
#include <fastrtps/rtps/writer/RTPSWriter.h>

#include <fastrtps/log/Log.h>
#include <fastrtps/utils/TimeConversion.h>

#include <fastrtps/rtps/resources/TimedEvent.h>
#include <fastrtps/rtps/messages/RTPSMessageCreator.h>
#include "../participant/RTPSParticipantImpl.h"

#include "rtps/RTPSDomainImpl.hpp"

#include <foonathan/memory/namespace_alias.hpp>
#include <fastrtps/utils/collections/foonathan_memory_helpers.hpp>

#if !defined(NDEBUG) && defined(FASTRTPS_SOURCE) && defined(__linux__)
#include <sys/syscall.h>
#include <unistd.h>
#include <pthread.h>
#include <cassert>
#include <mutex>
#endif

namespace eprosima {
namespace fastrtps {
namespace rtps {

WriterProxy::~WriterProxy()
{
    delete(initial_acknack_);
    delete(heartbeat_response_);
}

constexpr size_t changes_node_size = memory::set_node_size<std::pair<size_t, SequenceNumber_t> >::value;

WriterProxy::WriterProxy(
        StatefulReader* reader,
        const RemoteLocatorsAllocationAttributes& loc_alloc,
        const ResourceLimitedContainerConfig& changes_allocation)
    : reader_(reader)
    , attributes_(loc_alloc.max_unicast_locators, loc_alloc.max_multicast_locators)
    , heartbeat_response_(nullptr)
    , initial_acknack_(nullptr)
    , last_heartbeat_count_(0)
    , heartbeat_final_flag_(false)
    , is_alive_(false)
    , changes_pool_(
        changes_node_size,
        memory_pool_block_size<pool_allocator_t>(changes_node_size, changes_allocation))
    , changes_received_(changes_pool_)
    , guid_as_vector_(ResourceLimitedContainerConfig::fixed_size_configuration(1u))
    , guid_prefix_as_vector_(ResourceLimitedContainerConfig::fixed_size_configuration(1u))
    , is_on_same_process_(false)
{
    //Create Events
    heartbeat_response_ = new TimedEvent(reader_->getRTPSParticipant()->getEventResource(),
                    [&](TimedEvent::EventCode code) -> bool
                {
                    if (TimedEvent::EVENT_SUCCESS == code)
                    {
                        perform_heartbeat_response();
                    }

                    return false;
                }, 0);

    initial_acknack_ = new TimedEvent(reader_->getRTPSParticipant()->getEventResource(),
                    [&](TimedEvent::EventCode code) -> bool
                {
                    if (TimedEvent::EVENT_SUCCESS == code)
                    {
                        perform_initial_ack_nack();
                    }

                    return false;
                }, 0 );
    clear();
    logInfo(RTPS_READER, "Writer Proxy created in reader: " << reader_->getGuid().entityId);
}

void WriterProxy::start(
        const WriterProxyData& attributes,
        const SequenceNumber_t& initial_sequence)
{
#if !defined(NDEBUG) && defined(FASTRTPS_SOURCE) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    heartbeat_response_->update_interval(reader_->getTimes().heartbeatResponseDelay);
    initial_acknack_->update_interval(reader_->getTimes().initialAcknackDelay);

    attributes_ = attributes;
    guid_as_vector_.push_back(attributes_.guid());
    guid_prefix_as_vector_.push_back(attributes_.guid().guidPrefix);
    is_alive_ = true;
    is_on_same_process_ = RTPSDomainImpl::should_intraprocess_between(reader_->getGuid(), attributes_.guid());

    initial_acknack_->restart_timer();
    loaded_from_storage(initial_sequence);
}

void WriterProxy::update(
        const WriterProxyData& attributes)
{
#if !defined(NDEBUG) && defined(FASTRTPS_SOURCE) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    assert(is_alive_);
    attributes_ = attributes;
}

void WriterProxy::stop()
{
    initial_acknack_->cancel_timer();
    heartbeat_response_->cancel_timer();

    clear();
}

void WriterProxy::clear()
{
    is_alive_ = false;
    attributes_.guid(c_Guid_Unknown);
    last_heartbeat_count_ = 0;
    heartbeat_final_flag_.store(false);
    guid_as_vector_.clear();
    guid_prefix_as_vector_.clear();
    changes_received_.clear();
    is_on_same_process_ = false;
    loaded_from_storage(SequenceNumber_t());
}

void WriterProxy::loaded_from_storage(
        const SequenceNumber_t& seq_num)
{
    last_notified_ = seq_num;
    changes_from_writer_low_mark_ = seq_num;
    max_sequence_number_ = seq_num;
}

void WriterProxy::missing_changes_update(
        const SequenceNumber_t& seq_num)
{
#if !defined(NDEBUG) && defined(FASTRTPS_SOURCE) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    logInfo(RTPS_READER, attributes_.guid().entityId << ": changes up to seq_num: " << seq_num << " missing.");

    // Check was not removed from container.
    if (seq_num > changes_from_writer_low_mark_)
    {
        if (seq_num > max_sequence_number_)
        {
            max_sequence_number_ = seq_num;
        }
    }
}

void WriterProxy::lost_changes_update(
        const SequenceNumber_t& seq_num)
{
#if !defined(NDEBUG) && defined(FASTRTPS_SOURCE) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    logInfo(RTPS_READER, attributes_.guid().entityId << ": up to seq_num: " << seq_num);

    // Check was not removed from container.
    if (seq_num > changes_from_writer_low_mark_)
    {
        // Remove all received changes with a sequence lower than seq_num
        ChangeIterator it = std::lower_bound(changes_received_.begin(), changes_received_.end(), seq_num);
        changes_received_.erase(changes_received_.begin(), it);

        // Update low mark
        changes_from_writer_low_mark_ = seq_num - 1;
        if (changes_from_writer_low_mark_ > max_sequence_number_)
        {
            max_sequence_number_ = changes_from_writer_low_mark_;
        }

        // Next could need to be removed.
        cleanup();
    }
}

bool WriterProxy::received_change_set(
        const SequenceNumber_t& seq_num)
{
    logInfo(RTPS_READER, attributes_.guid().entityId << ": seq_num: " << seq_num);
    return received_change_set(seq_num, true);
}

bool WriterProxy::irrelevant_change_set(
        const SequenceNumber_t& seq_num)
{
    return received_change_set(seq_num, false);
}

bool WriterProxy::received_change_set(
        const SequenceNumber_t& seq_num,
        bool /* is_relevance */ )
{
#if !defined(NDEBUG) && defined(FASTRTPS_SOURCE) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    // Check if CacheChange_t was already and it was already removed from changesFromW container.
    if (seq_num <= changes_from_writer_low_mark_)
    {
        logInfo(RTPS_READER, "Change " << seq_num << " <= than max available sequence number "
                                       << changes_from_writer_low_mark_);
        return false;
    }

    // If will be the last element, insert it at the end.
    if (seq_num > max_sequence_number_)
    {
        // If it is the next to be acknowledeg, not insert
        if (seq_num == changes_from_writer_low_mark_ + 1)
        {
            changes_from_writer_low_mark_ = seq_num;
        }
        else
        {
            changes_received_.insert(changes_received_.end(), seq_num);
        }
        max_sequence_number_ = seq_num;
    }
    else
    {
        // Check if it is next to the last acknowledged
        if (changes_from_writer_low_mark_ + 1 == seq_num)
        {
            changes_from_writer_low_mark_ = seq_num;
            cleanup();
        }
        else
        {
            // Check if already received
            if (changes_received_.find(seq_num) != changes_received_.end())
            {
                return false;
            }

            changes_received_.insert(seq_num);
        }
    }

    return true;
}

SequenceNumberSet_t WriterProxy::missing_changes() const
{
#if defined(__DEBUG) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    SequenceNumber_t first_missing = changes_from_writer_low_mark_ + 1;
    SequenceNumber_t max_missing = std::min(first_missing + 256UL, max_sequence_number_ + 1);
    SequenceNumberSet_t sns(first_missing);

    for (SequenceNumber_t seq : changes_received_)
    {
        seq = std::min(seq, max_missing);
        sns.add_range(first_missing, seq);
        first_missing = seq + 1;
        if (first_missing >= max_missing)
        {
            break;
        }
    }

    if (first_missing < max_missing)
    {
        sns.add_range(first_missing, max_missing);
    }

    return sns;
}

bool WriterProxy::change_was_received(
        const SequenceNumber_t& seq_num) const
{
#if defined(__DEBUG) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    if (seq_num <= changes_from_writer_low_mark_)
    {
        return true;
    }

    ChangeIterator chit = changes_received_.find(seq_num);
    return chit != changes_received_.end();
}

const SequenceNumber_t WriterProxy::available_changes_max() const
{
#if defined(__DEBUG) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    return changes_from_writer_low_mark_;
}

void WriterProxy::change_removed_from_history(
        const SequenceNumber_t& seq_num)
{
#if defined(__DEBUG) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    // Check sequence number is in the container, because it was not clean up.
    if (seq_num <= changes_from_writer_low_mark_)
    {
        return;
    }

    ChangeIterator chit = changes_received_.find(seq_num);

    (void)chit;

    // Element must be in the container. In other case, bug.
    assert(chit != changes_received_.end());

    // Previously, it was asserted that the change couldn't be the first and should have RECEIVED
    // status. As we only keep received changes now, status is already checked by the previous assert.
    // It can now be the case that the change being removed is the first if there are missing changes
    // in the (changes_from_writer_low_mark_, seq_num) range.

    // We are removing a change that will not be notified to the user. This may be due to the following:
    // a) history became full (either due to KEEP_LAST or RESOURCE_LIMITS)
    // b) lifespan timer expired for seq_num
    // Previously, change was marked as irrelevant.

    // As this may imply that all changes with a lower sequence number will also be dropped,
    // now that history is full, it may be interesting to act as if a heartbeat with an initial
    // sequence of seq_num has been received, i.e. calling lost_changes_update(seq_num).
    // If we don't do it, changes_received_ may grow above the limits stablished for it,
    // thus causing undesired dynamic allocations.

    // For case a) a call to lost_changes_update is done inside StatefulReader::change_received.
    // For case b) it does not imply a dynamic allocation problem.
}

void WriterProxy::cleanup()
{
    ChangeIterator chit = changes_received_.begin();

    // Jump over all consecutive received changes starting on the next to low_mark
    while (chit != changes_received_.end() && *chit == changes_from_writer_low_mark_ + 1)
    {
        chit++;
        changes_from_writer_low_mark_++;
    }

    // Remove all those changes
    changes_received_.erase(changes_received_.begin(), chit);
}

bool WriterProxy::are_there_missing_changes() const
{
#if defined(__DEBUG) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    return changes_from_writer_low_mark_ < max_sequence_number_;
}

size_t WriterProxy::unknown_missing_changes_up_to(
        const SequenceNumber_t& seq_num) const
{
#if defined(__DEBUG) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    uint32_t returnedValue = 0;

    if (seq_num > changes_from_writer_low_mark_)
    {
        SequenceNumber_t first_missing = changes_from_writer_low_mark_ + 1;
        SequenceNumber_t max_missing = std::min(seq_num, max_sequence_number_ + 1);
        SequenceNumberSet_t sns(first_missing);
        SequenceNumberDiff d_fun;

        for (SequenceNumber_t seq : changes_received_)
        {
            seq = std::min(seq, max_missing);
            if (first_missing < seq)
            {
                returnedValue += d_fun(seq, first_missing);
            }
            first_missing = seq + 1;
            if (first_missing >= max_missing)
            {
                break;
            }
        }

        if (first_missing < max_missing)
        {
            returnedValue += d_fun(max_missing, first_missing);
        }
    }

    return returnedValue;
}

size_t WriterProxy::number_of_changes_from_writer() const
{
#if defined(__DEBUG) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    if (max_sequence_number_ > changes_from_writer_low_mark_)
    {
        SequenceNumberDiff d_fun;
        return d_fun(max_sequence_number_, changes_from_writer_low_mark_);
    }

    return 0;
}

SequenceNumber_t WriterProxy::next_cache_change_to_be_notified()
{
#if defined(__DEBUG) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    if (last_notified_ < changes_from_writer_low_mark_)
    {
        ++last_notified_;
        return last_notified_;
    }

    return SequenceNumber_t::unknown();
}

void WriterProxy::perform_initial_ack_nack() const
{
    // Send initial NACK.
    SequenceNumberSet_t sns(SequenceNumber_t(0, 0));
    if (is_on_same_process_)
    {
        RTPSWriter* writer = RTPSDomainImpl::find_local_writer(attributes_.guid());
        if (writer)
        {
            bool tmp;
            writer->process_acknack(attributes_.guid(), reader_->getGuid(), 1, SequenceNumberSet_t(), false, tmp);
        }
    }
    else
    {
        reader_->send_acknack(this, sns, *this, false);
    }
}

void WriterProxy::perform_heartbeat_response() const
{
    reader_->send_acknack(this, *this, heartbeat_final_flag_.load());
}

bool WriterProxy::process_heartbeat(
        uint32_t count,
        const SequenceNumber_t& first_seq,
        const SequenceNumber_t& last_seq,
        bool final_flag,
        bool liveliness_flag,
        bool disable_positive,
        bool& assert_liveliness)
{
#if defined(__DEBUG) && defined(__linux__)
    assert(get_mutex_owner() == get_thread_id());
#endif

    assert_liveliness = false;
    if (last_heartbeat_count_ < count)
    {
        // If it is the first heartbeat message, we can try to cancel initial ack.
        // TODO: This timer cancelling should be checked if needed with the liveliness implementation.
        // To keep PARTICIPANT_DROPPED event we should add an explicit participant_liveliness QoS.
        // This is now commented to avoid issues #457 and #155
        // initial_acknack_->cancel_timer();

        last_heartbeat_count_ = count;
        lost_changes_update(first_seq);
        missing_changes_update(last_seq);
        heartbeat_final_flag_.store(final_flag);

        //Analyze whether a acknack message is needed:
        if (!is_on_same_process_)
        {
            if (!final_flag)
            {
                if (!disable_positive || are_there_missing_changes())
                {
                    heartbeat_response_->restart_timer();
                }
            }
            else if (final_flag && !liveliness_flag)
            {
                if (are_there_missing_changes())
                {
                    heartbeat_response_->restart_timer();
                }
            }
            else
            {
                assert_liveliness = liveliness_flag;
            }
        }
        else
        {
            assert_liveliness = liveliness_flag;
        }

        return true;
    }

    return false;
}

void WriterProxy::update_heartbeat_response_interval(
        const Duration_t& interval)
{
    heartbeat_response_->update_interval(interval);
}

bool WriterProxy::send(
        CDRMessage_t* message,
        std::chrono::steady_clock::time_point& max_blocking_time_point) const
{
    if (is_on_same_process_)
    {
        return true;
    }

    for (const Locator_t& locator : remote_locators_shrinked())
    {
        if (!reader_->send_sync_nts(message, locator, max_blocking_time_point))
        {
            return false;
        }
    }

    return true;
}

#if !defined(NDEBUG) && defined(FASTRTPS_SOURCE) && defined(__linux__)
int WriterProxy::get_mutex_owner() const
{
    auto mutex = reader_->getMutex().native_handle();
    return mutex->__data.__owner;
}

int WriterProxy::get_thread_id() const
{
    return syscall(__NR_gettid);
}

#endif

} /* namespace rtps */
} /* namespace fastrtps */
} /* namespace eprosima */
