// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#if defined(__linux__) && !defined(__ANDROID__)

#include "./impl/linux.cpp"
#include "./impl/unix_common.cpp"

#elif defined(__APPLE__)

#include "./impl/apple.cpp"
#include "./impl/unix_common.cpp"

// #elif defined(_WIN32)

// TODO(wjwwood): install custom malloc (and others) for Windows.

#else

#include "./impl/unsupported_os.cpp"

#endif  // if defined(__linux__) elif defined(__APPLE__) elif defined(_WIN32) else ...
