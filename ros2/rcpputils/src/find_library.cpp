// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "rcpputils/find_library.hpp"

#include <cassert>
#include <cstddef>

#include <list>
#include <fstream>
#include <sstream>
#include <string>

#include "rcutils/filesystem.h"
#include "rcutils/get_env.h"

namespace rcpputils
{

namespace
{

#ifdef _WIN32
static constexpr char kPathVar[] = "PATH";
static constexpr char kPathSeparator = ';';
static constexpr char kSolibPrefix[] = "";
static constexpr char kSolibExtension[] = ".dll";
#elif __APPLE__
static constexpr char kPathVar[] = "DYLD_LIBRARY_PATH";
static constexpr char kPathSeparator = ':';
static constexpr char kSolibPrefix[] = "lib";
static constexpr char kSolibExtension[] = ".dylib";
#else
static constexpr char kPathVar[] = "LD_LIBRARY_PATH";
static constexpr char kPathSeparator = ':';
static constexpr char kSolibPrefix[] = "lib";
static constexpr char kSolibExtension[] = ".so";
#endif

std::string get_env_var(const char * env_var)
{
  const char * value{};
  const char * err = rcutils_get_env(env_var, &value);
  if (err) {
    throw std::runtime_error(err);
  }
  return value ? value : "";
}

std::list<std::string> split(const std::string & value, const char delimiter)
{
  std::list<std::string> list;
  std::istringstream ss(value);
  std::string s;
  while (std::getline(ss, s, delimiter)) {
    list.push_back(s);
  }
  return list;
}

}  // namespace

std::string find_library_path(const std::string & library_name)
{
  std::string search_path = get_env_var(kPathVar);
  std::list<std::string> search_paths = split(search_path, kPathSeparator);

  std::string filename = kSolibPrefix;
  filename += library_name + kSolibExtension;

  for (const auto & search_path : search_paths) {
    std::string path = search_path + "/" + filename;
    if (rcutils_is_file(path.c_str())) {
      return path;
    }
  }
  return "";
}

}  // namespace rcpputils
