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

#ifndef TYPE_SUPPORT_DISPATCH_HPP_
#define TYPE_SUPPORT_DISPATCH_HPP_

#include <cstddef>
#include <cstdio>
#include <cstring>

#include <memory>
#include <stdexcept>
#include <list>
#include <string>

#include "rcpputils/find_library.hpp"
#include "rcpputils/shared_library.hpp"
#include "rcutils/error_handling.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/type_support_map.h"

namespace rosidl_typesupport_c
{

extern const char * typesupport_identifier;

template<typename TypeSupport>
const TypeSupport *
get_typesupport_handle_function(
  const TypeSupport * handle, const char * identifier)
{
  if (strcmp(handle->typesupport_identifier, identifier) == 0) {
    return handle;
  }

  if (handle->typesupport_identifier == rosidl_typesupport_c__typesupport_identifier) {
    const type_support_map_t * map = \
      static_cast<const type_support_map_t *>(handle->data);
    for (size_t i = 0; i < map->size; ++i) {
      if (strcmp(map->typesupport_identifier[i], identifier) != 0) {
        continue;
      }
      rcpputils::SharedLibrary * lib = nullptr;

      if (!map->data[i]) {
        char library_name[1024];
        snprintf(
          library_name, 1023, "%s__%s",
          map->package_name, identifier);

        std::string library_path;
        try {
          library_path = rcpputils::find_library_path(library_name);
        } catch (const std::exception & e) {
          RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
            "Failed to find library '%s' due to %s\n",
            library_name, e.what());
          return nullptr;
        }

        if (library_path.empty()) {
          RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
            "Failed to find library '%s'\n", library_name);
          return nullptr;
        }

        try {
          lib = new rcpputils::SharedLibrary(library_path.c_str());
        } catch (const std::runtime_error & e) {
          RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
            "Could not load library %s: %s\n", library_path.c_str(), e.what());
          return nullptr;
        } catch (const std::bad_alloc & e) {
          RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
            "Could not load library %s: %s\n", library_path.c_str(), e.what());
          return nullptr;
        }
        map->data[i] = lib;
      }
      auto clib = static_cast<const rcpputils::SharedLibrary *>(map->data[i]);
      lib = const_cast<rcpputils::SharedLibrary *>(clib);

      void * sym = nullptr;

      try {
        if (!lib->has_symbol(map->symbol_name[i])) {
          RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
            "Failed to find symbol '%s' in library\n", map->symbol_name[i]);
          return nullptr;
        }
        sym = lib->get_symbol(map->symbol_name[i]);
      } catch (const std::exception & e) {
        RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
          "Failed to get symbol '%s' in library: %s\n",
          map->symbol_name[i], e.what());
        return nullptr;
      }

      typedef const TypeSupport * (* funcSignature)(void);
      funcSignature func = reinterpret_cast<funcSignature>(sym);
      const TypeSupport * ts = func();
      return ts;
    }
  }
  RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
    "Handle's typesupport identifier (%s) is not supported by this library\n",
    handle->typesupport_identifier);
  return nullptr;
}

}  // namespace rosidl_typesupport_c

#endif  // TYPE_SUPPORT_DISPATCH_HPP_
