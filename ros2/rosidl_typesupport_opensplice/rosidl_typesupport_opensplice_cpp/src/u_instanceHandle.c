//
//                         OpenSplice DDS
//
// Copyright 2006-2017 PrismTech Limited, its affiliated companies and licensors.
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

#include "rosidl_typesupport_opensplice_cpp/u__instanceHandle.h"

#define HANDLE_GLOBAL_MASK (0x80000000)

typedef union {
  struct
  {
    c_ulong lifecycleId;
    c_ulong localId;
  } lid;
  u_instanceHandle handle;
} u_instanceHandleTranslator;

v_gid
u_instanceHandleToGID(
  u_instanceHandle handle)
{
  u_instanceHandleTranslator translator;
  v_gid gid;

  translator.handle = handle;

  gid.systemId = translator.lid.lifecycleId & ~HANDLE_GLOBAL_MASK;
  gid.localId = translator.lid.localId;
  gid.serial = 0;

  return gid;
}
