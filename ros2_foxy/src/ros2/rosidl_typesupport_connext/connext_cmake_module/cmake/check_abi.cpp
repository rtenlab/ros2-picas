// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#define _GLIBCXX_USE_CXX11_ABI 0

#include <ndds/connext_cpp/connext_cpp_requester_details.h>

#if __GNUC__ >= 9
# pragma GCC diagnostic ignored "-Wclass-memaccess"
#endif
#pragma GCC diagnostic ignored "-Wuninitialized"

int main(int, char **)
{
  DDSDomainParticipantFactory * dpf = DDSDomainParticipantFactory::get_instance();

  DDS_DomainParticipantQos qos;
  dpf->get_default_participant_qos(qos);

  DDS_DomainId_t domain = static_cast<DDS_DomainId_t>(1);

  DDSDomainParticipant * p = dpf->create_participant(
    domain, qos, NULL, DDS_STATUS_MASK_NONE);

  connext::RequesterParams params(p);
  params.service_name("foo");
}
