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

#include <gtest/gtest.h>

#include <string>

#include "rcpputils/filesystem_helper.hpp"

#ifdef _WIN32
static constexpr const bool is_win32 = true;
#else
static constexpr const bool is_win32 = false;
#endif

using path = rcpputils::fs::path;

std::string build_extension_path()
{
  return is_win32 ? R"(C:\foo\bar\baz.yml)" : "/bar/foo/baz.yml";
}

std::string build_double_extension_path()
{
  return is_win32 ? R"(C:\bar\baz.bar.yml)" : "/foo/baz.bar.yml";
}

std::string build_no_extension_path()
{
  return is_win32 ? R"(C:\bar\baz)" : "/bar/baz";
}

std::string build_directory_path()
{
  return is_win32 ? R"(.\test_folder)" : R"(./test_folder)";
}

TEST(TestFilesystemHelper, join_path)
{
  auto p = path("foo") / path("bar");

  if (is_win32) {
    EXPECT_EQ("foo\\bar", p.string());
  } else {
    EXPECT_EQ("foo/bar", p.string());
  }
}

TEST(TestFilesystemHelper, to_native_path)
{
  {
    auto p = path("/foo/bar/baz");
    if (is_win32) {
      EXPECT_EQ("\\foo\\bar\\baz", p.string());
    } else {
      EXPECT_EQ("/foo/bar/baz", p.string());
    }
  }
  {
    auto p = path("\\foo\\bar\\baz");
    if (is_win32) {
      EXPECT_EQ("\\foo\\bar\\baz", p.string());
    } else {
      EXPECT_EQ("/foo/bar/baz", p.string());
    }
  }
  {
    auto p = path("/foo//bar/baz");
    if (is_win32) {
      EXPECT_EQ("\\foo\\\\bar\\baz", p.string());
    } else {
      EXPECT_EQ("/foo//bar/baz", p.string());
    }
  }
  {
    auto p = path("\\foo\\\\bar\\baz");
    if (is_win32) {
      EXPECT_EQ("\\foo\\\\bar\\baz", p.string());
    } else {
      EXPECT_EQ("/foo//bar/baz", p.string());
    }
  }
}

TEST(TestFilesystemHelper, is_absolute)
{
  if (is_win32) {
    {
      auto p = path("C:\\foo\\bar\\baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("D:\\foo\\bar\\baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("C:/foo/bar/baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("foo/bar/baz");
      EXPECT_FALSE(p.is_absolute());
    }
  } else {
    {
      auto p = path("/foo/bar/baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("foo/bar/baz");
      EXPECT_FALSE(p.is_absolute());
    }
  }
}

TEST(TestFilesystemHelper, correct_extension)
{
  {
    auto p = path(build_extension_path());
    auto ext = p.extension();
    EXPECT_EQ(".yml", ext.string());
  }
  {
    auto p = path(build_double_extension_path());
    auto ext = p.extension();
    EXPECT_EQ(".yml", ext.string());
  }
  {
    auto p = path(build_no_extension_path());
    auto ext = p.extension();
    EXPECT_EQ("", ext.string());
  }
}

TEST(TestFilesystemHelper, is_empty)
{
  auto p = path("");
  EXPECT_TRUE(p.empty());
}

/**
 * Test create directories.
 *
 * NOTE: expects the current directory to be write-only, else test will fail.
 *
 */
TEST(TestFilesystemHelper, create_directories)
{
  auto p = path(build_directory_path());
  EXPECT_TRUE(rcpputils::fs::create_directories(p));
}

TEST(TestFilesystemHelper, remove_extension)
{
  auto p = path("foo.txt");
  p = rcpputils::fs::remove_extension(p.string());
  EXPECT_EQ("foo", p.string());
}

TEST(TestFilesystemHelper, remove_extensions)
{
  auto p = path("foo.txt.compress");
  p = rcpputils::fs::remove_extension(p, 2);
  EXPECT_EQ("foo", p.string());
}

TEST(TestFilesystemHelper, remove_extensions_overcount)
{
  auto p = path("foo.txt.compress");
  p = rcpputils::fs::remove_extension(p, 4);
  EXPECT_EQ("foo", p.string());
}

TEST(TestFilesystemHelper, remove_extension_no_extension)
{
  auto p = path("foo");
  p = rcpputils::fs::remove_extension(p);
  EXPECT_EQ("foo", p.string());
}
