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

#include "type_support_dispatch.hpp"

#include <list>
#include <string>

#include <fstream>
#include <sstream>

namespace rosidl_typesupport_c
{

std::string find_library_path(const std::string & library_name)
{
  const char * env_var;
  char separator;
  const char * filename_prefix;
  const char * filename_extension;
#ifdef _WIN32
  env_var = "PATH";
  separator = ';';
  filename_prefix = "";
  filename_extension = ".dll";
#elif __APPLE__
  env_var = "DYLD_LIBRARY_PATH";
  separator = ':';
  filename_prefix = "lib";
  filename_extension = ".dylib";
#else
  env_var = "LD_LIBRARY_PATH";
  separator = ':';
  filename_prefix = "lib";
  filename_extension = ".so";
#endif
  std::string search_path = get_env_var(env_var);
  std::list<std::string> search_paths = split(search_path, separator);

  std::string filename = filename_prefix;
  filename += library_name + filename_extension;

  for (auto it : search_paths) {
    std::string path = it + "/" + filename;
    if (is_file_exist(path.c_str())) {
      return path;
    }
  }
  return "";
}

std::string get_env_var(const char * env_var)
{
  char * value = nullptr;
#ifndef _WIN32
  value = getenv(env_var);
#else
  size_t value_size;
  _dupenv_s(&value, &value_size, env_var);
#endif
  std::string value_str = "";
  if (value) {
    value_str = value;
#ifdef _WIN32
    free(value);
#endif
  }
  // printf("get_env_var(%s) = %s\n", env_var, value_str.c_str());
  return value_str;
}

std::list<std::string> split(const std::string & value, const char delimiter)
{
  std::list<std::string> list;
  std::istringstream ss(value);
  std::string s;
  while (std::getline(ss, s, delimiter)) {
    list.push_back(s);
  }
  // printf("split(%s) = %zu\n", value.c_str(), list.size());
  return list;
}

bool is_file_exist(const char * filename)
{
  std::ifstream h(filename);
  // printf("is_file_exist(%s) = %s\n", filename, h.good() ? "true" : "false");
  return h.good();
}

}  // namespace rosidl_typesupport_c
