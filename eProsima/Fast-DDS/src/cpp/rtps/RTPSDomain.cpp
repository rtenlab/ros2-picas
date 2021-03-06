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

/*
 * RTPSDomain.cpp
 *
 */

#include <fastrtps/rtps/RTPSDomain.h>

#include <fastrtps/rtps/participant/RTPSParticipant.h>
#include "participant/RTPSParticipantImpl.h"

#include <fastrtps/log/Log.h>

#include <fastrtps/transport/UDPv4Transport.h>
#include <fastrtps/transport/UDPv6Transport.h>
#include <fastrtps/transport/test_UDPv4Transport.h>

#include <fastrtps/utils/IPFinder.h>
#include <fastrtps/utils/IPLocator.h>
#include <fastrtps/utils/System.h>
#include <fastrtps/utils/md5.h>

#include <fastrtps/rtps/writer/RTPSWriter.h>
#include <fastrtps/rtps/reader/RTPSReader.h>

#include <fastrtps/xmlparser/XMLProfileManager.h>

#include "RTPSDomainImpl.hpp"

#include <chrono>
#include <thread>

namespace eprosima {
namespace fastrtps {
namespace rtps {

std::mutex RTPSDomain::m_mutex;
std::atomic<uint32_t> RTPSDomain::m_maxRTPSParticipantID(1);
std::vector<RTPSDomain::t_p_RTPSParticipant> RTPSDomain::m_RTPSParticipants;
std::set<uint32_t> RTPSDomain::m_RTPSParticipantIDs;

void RTPSDomain::stopAll()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    logInfo(RTPS_PARTICIPANT, "DELETING ALL ENDPOINTS IN THIS DOMAIN");

    while (m_RTPSParticipants.size() > 0)
    {
        RTPSDomain::t_p_RTPSParticipant participant = m_RTPSParticipants.back();
        m_RTPSParticipantIDs.erase(m_RTPSParticipantIDs.find(participant.second->getRTPSParticipantID()));
        m_RTPSParticipants.pop_back();

        lock.unlock();
        RTPSDomain::removeRTPSParticipant_nts(participant);
        lock.lock();
    }
    logInfo(RTPS_PARTICIPANT, "RTPSParticipants deleted correctly ");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

RTPSParticipant* RTPSDomain::createParticipant(
        const RTPSParticipantAttributes& attrs,
        RTPSParticipantListener* listen)
{
    logInfo(RTPS_PARTICIPANT, "");

    RTPSParticipantAttributes PParam = attrs;

    if (PParam.builtin.discovery_config.leaseDuration < c_TimeInfinite &&
            PParam.builtin.discovery_config.leaseDuration <=
            PParam.builtin.discovery_config.leaseDuration_announcementperiod)                                                                                                               //TODO CHeckear si puedo ser infinito
    {
        logError(RTPS_PARTICIPANT,
                "RTPSParticipant Attributes: LeaseDuration should be >= leaseDuration announcement period");
        return nullptr;
    }

    uint32_t ID;
    {
        std::lock_guard<std::mutex> guard(m_mutex);

        if (PParam.participantID < 0)
        {
            ID = getNewId();
            while (m_RTPSParticipantIDs.insert(ID).second == false)
            {
                ID = getNewId();
            }
        }
        else
        {
            ID = PParam.participantID;
            if (m_RTPSParticipantIDs.insert(ID).second == false)
            {
                logError(RTPS_PARTICIPANT, "RTPSParticipant with the same ID already exists");
                return nullptr;
            }
        }
    }

    if (!PParam.defaultUnicastLocatorList.isValid())
    {
        logError(RTPS_PARTICIPANT, "Default Unicast Locator List contains invalid Locator");
        return nullptr;
    }
    if (!PParam.defaultMulticastLocatorList.isValid())
    {
        logError(RTPS_PARTICIPANT, "Default Multicast Locator List contains invalid Locator");
        return nullptr;
    }

    PParam.participantID = ID;
    LocatorList_t loc;
    IPFinder::getIP4Address(&loc);

    if (loc.empty() && PParam.builtin.initialPeersList.empty())
    {
        Locator_t local;
        IPLocator::setIPv4(local, 127, 0, 0, 1);
        PParam.builtin.initialPeersList.push_back(local);
    }

    // Generate a new GuidPrefix_t
    GuidPrefix_t guidP;
    {
        // Make a new participant GuidPrefix_t up
        int pid = System::GetPID();

        guidP.value[0] = c_VendorId_eProsima[0];
        guidP.value[1] = c_VendorId_eProsima[1];

        if (loc.size() > 0)
        {
            MD5 md5;
            for (auto& l : loc)
            {
                md5.update(l.address, sizeof(l.address));
            }
            md5.finalize();
            uint16_t hostid = 0;
            for (size_t i = 0; i < sizeof(md5.digest); i += 2)
            {
                hostid ^= ((md5.digest[i] << 8) | md5.digest[i + 1]);
            }
            guidP.value[2] = octet(hostid);
            guidP.value[3] = octet(hostid >> 8);
        }
        else
        {
            guidP.value[2] = 127;
            guidP.value[3] = 1;
        }
        guidP.value[4] = octet(pid);
        guidP.value[5] = octet(pid >> 8);
        guidP.value[6] = octet(pid >> 16);
        guidP.value[7] = octet(pid >> 24);
        guidP.value[8] = octet(ID);
        guidP.value[9] = octet(ID >> 8);
        guidP.value[10] = octet(ID >> 16);
        guidP.value[11] = octet(ID >> 24);
    }

    RTPSParticipant* p = new RTPSParticipant(nullptr);
    RTPSParticipantImpl* pimpl = nullptr;

    // If we force the participant to have a specific prefix we must define a different persistence GuidPrefix_t that
    // would ensure builtin endpoints are able to differentiate between a communication loss and a participant recovery
    if (PParam.prefix != c_GuidPrefix_Unknown)
    {
        pimpl = new RTPSParticipantImpl(PParam, PParam.prefix, guidP, p, listen);
    }
    else
    {
        pimpl = new RTPSParticipantImpl(PParam, guidP, p, listen);
    }

#if HAVE_SECURITY
    // Check security was correctly initialized
    if (!pimpl->is_security_initialized())
    {
        logError(RTPS_PARTICIPANT, "Cannot create participant due to security initialization error");
        delete pimpl;
        return nullptr;
    }
#endif

    // Check there is at least one transport registered.
    if (!pimpl->networkFactoryHasRegisteredTransports())
    {
        logError(RTPS_PARTICIPANT, "Cannot create participant, because there is any transport");
        delete pimpl;
        return nullptr;
    }

    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_RTPSParticipants.push_back(t_p_RTPSParticipant(p, pimpl));
    }

    // Start protocols
    pimpl->enable();
    return p;
}

bool RTPSDomain::removeRTPSParticipant(
        RTPSParticipant* p)
{
    if (p != nullptr)
    {
        p->mp_impl->disable();

        std::unique_lock<std::mutex> lock(m_mutex);
        for (auto it = m_RTPSParticipants.begin(); it != m_RTPSParticipants.end(); ++it)
        {
            if (it->second->getGuid().guidPrefix == p->getGuid().guidPrefix)
            {
                RTPSDomain::t_p_RTPSParticipant participant = *it;
                m_RTPSParticipants.erase(it);
                m_RTPSParticipantIDs.erase(m_RTPSParticipantIDs.find(participant.second->getRTPSParticipantID()));
                lock.unlock();
                removeRTPSParticipant_nts(participant);
                return true;
            }
        }
    }
    logError(RTPS_PARTICIPANT, "RTPSParticipant not valid or not recognized");
    return false;
}

void RTPSDomain::removeRTPSParticipant_nts(
        RTPSDomain::t_p_RTPSParticipant& participant)
{
    delete(participant.second);
}

RTPSWriter* RTPSDomain::createRTPSWriter(
        RTPSParticipant* p,
        WriterAttributes& watt,
        WriterHistory* hist,
        WriterListener* listen)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    for (auto it = m_RTPSParticipants.begin(); it != m_RTPSParticipants.end(); ++it)
    {
        if (it->first->getGuid().guidPrefix == p->getGuid().guidPrefix)
        {
            t_p_RTPSParticipant participant = *it;
            lock.unlock();
            RTPSWriter* writ;
            if (participant.second->createWriter(&writ, watt, hist, listen))
            {
                return writ;
            }
            return nullptr;
        }
    }
    return nullptr;
}

bool RTPSDomain::removeRTPSWriter(
        RTPSWriter* writer)
{
    if (writer != nullptr)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        for (auto it = m_RTPSParticipants.begin(); it != m_RTPSParticipants.end(); ++it)
        {
            if (it->first->getGuid().guidPrefix == writer->getGuid().guidPrefix)
            {
                t_p_RTPSParticipant participant = *it;
                lock.unlock();
                return participant.second->deleteUserEndpoint((Endpoint*)writer);
            }
        }
    }
    return false;
}

RTPSReader* RTPSDomain::createRTPSReader(
        RTPSParticipant* p,
        ReaderAttributes& ratt,
        ReaderHistory* rhist,
        ReaderListener* rlisten)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    for (auto it = m_RTPSParticipants.begin(); it != m_RTPSParticipants.end(); ++it)
    {
        if (it->first->getGuid().guidPrefix == p->getGuid().guidPrefix)
        {
            t_p_RTPSParticipant participant = *it;
            lock.unlock();
            RTPSReader* reader;
            if (participant.second->createReader(&reader, ratt, rhist, rlisten))
            {
                return reader;
            }

            return nullptr;
        }
    }
    return nullptr;
}

bool RTPSDomain::removeRTPSReader(
        RTPSReader* reader)
{
    if (reader !=  nullptr)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        for (auto it = m_RTPSParticipants.begin(); it != m_RTPSParticipants.end(); ++it)
        {
            if (it->first->getGuid().guidPrefix == reader->getGuid().guidPrefix)
            {
                t_p_RTPSParticipant participant = *it;
                lock.unlock();
                return participant.second->deleteUserEndpoint((Endpoint*)reader);
            }
        }
    }
    return false;
}

RTPSReader* RTPSDomainImpl::find_local_reader(
        const GUID_t& reader_guid)
{
    std::lock_guard<std::mutex> guard(RTPSDomain::m_mutex);
    for (const RTPSDomain::t_p_RTPSParticipant& participant : RTPSDomain::m_RTPSParticipants)
    {
        if (participant.second->getGuid().guidPrefix == reader_guid.guidPrefix)
        {
            // Participant found, forward the query
            return participant.second->find_local_reader(reader_guid);
        }
    }

    return nullptr;
}

RTPSWriter* RTPSDomainImpl::find_local_writer(
        const GUID_t& writer_guid)
{
    std::lock_guard<std::mutex> guard(RTPSDomain::m_mutex);
    for (const RTPSDomain::t_p_RTPSParticipant& participant : RTPSDomain::m_RTPSParticipants)
    {
        if (participant.second->getGuid().guidPrefix == writer_guid.guidPrefix)
        {
            // Participant found, forward the query
            return participant.second->find_local_writer(writer_guid);
        }
    }

    return nullptr;
}

/**
 * Check whether intraprocess delivery should be used between two GUIDs.
 *
 * @param local_guid    GUID of the local endpoint performing the query.
 * @param matched_guid  GUID being queried about.
 *
 * @returns true when intraprocess delivery is enabled, false otherwise.
 */
bool RTPSDomainImpl::should_intraprocess_between(
        const GUID_t& local_guid,
        const GUID_t& matched_guid)
{
    if (!local_guid.is_on_same_process_as(matched_guid))
    {
        // Not on the same process, should not use intraprocess mechanism.
        return false;
    }

    if (local_guid.entityId == c_EntityId_SPDPWriter || local_guid.entityId == c_EntityId_SPDPReader)
    {
        // Always disabled for PDP, to avoid inter-domain communications.
        return false;
    }

    switch (xmlparser::XMLProfileManager::library_settings().intraprocess_delivery)
    {
        case IntraprocessDeliveryType::INTRAPROCESS_FULL:
            return true;

        case IntraprocessDeliveryType::INTRAPROCESS_USER_DATA_ONLY:
            return !matched_guid.is_builtin();

        case IntraprocessDeliveryType::INTRAPROCESS_OFF:
        default:
            break;
    }

    return false;
}

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima
