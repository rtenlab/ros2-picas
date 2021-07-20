// Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef GUID_HPP_
#define GUID_HPP_

#include <dds_dcps.h>


inline DDS::InstanceHandle_t DDS_BuiltinTopicKey_to_InstanceHandle(
  DDS::BuiltinTopicKey_t builtinTopicKey)
{
  v_builtinTopicKey gid;

  // the following logic came from copyInTopicKey() in opensplice source code
  gid.systemId = builtinTopicKey[0];
  gid.localId = builtinTopicKey[1];
  gid.serial = builtinTopicKey[2];

  return u_instanceHandleFromGID(gid);
}

#endif  // GUID_HPP_
