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

#include <fstream>
#include <string>

#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/get_env.hpp"

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
  {
    auto p = path("foo") / path("bar");
    if (is_win32) {
      EXPECT_EQ("foo\\bar", p.string());
    } else {
      EXPECT_EQ("foo/bar", p.string());
    }
  }

  if (is_win32) {
    auto p = path("foo") / path("C:\\bar");
    EXPECT_EQ("C:\\bar", p.string());
  } else {
    auto p = path("foo") / path("/bar");
    EXPECT_EQ("/bar", p.string());
  }
}

TEST(TestFilesystemHelper, parent_path)
{
  {
    auto p = path("my") / path("path");
    EXPECT_EQ(p.parent_path().string(), path("my").string());
  }
  {
    auto p = path("foo");
    EXPECT_EQ(p.parent_path().string(), ".");
  }
  {
    if (is_win32) {
      {
        auto p = path("C:\\foo");
        EXPECT_EQ(p.parent_path().string(), "C:\\");
      }
      {
        auto p = path("\\foo");
        EXPECT_EQ(p.parent_path().string(), "\\");
      }
    } else {
      auto p = path("/foo");
      EXPECT_EQ(p.parent_path().string(), "/");
    }
  }
  {
    if (is_win32) {
      {
        auto p = path("C:\\");
        EXPECT_EQ(p.parent_path().string(), "C:\\");
      }
      {
        auto p = path("\\");
        EXPECT_EQ(p.parent_path().string(), "\\");
      }
    } else {
      auto p = path("/");
      EXPECT_EQ(p.parent_path().string(), "/");
    }
  }
  {
    auto p = path("");
    EXPECT_EQ(p.parent_path().string(), "");
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
      auto p = path("\\foo\\bar\\baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("/foo/bar/baz");
      EXPECT_TRUE(p.is_absolute());
    }
    {
      auto p = path("foo/bar/baz");
      EXPECT_FALSE(p.is_absolute());
    }
    {
      auto p = path("C:");
      EXPECT_FALSE(p.is_absolute());
    }
    {
      auto p = path("C");
      EXPECT_FALSE(p.is_absolute());
    }
    {
      auto p = path("");
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
    {
      auto p = path("f");
      EXPECT_FALSE(p.is_absolute());
    }
    {
      auto p = path("");
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

TEST(TestFilesystemHelper, exists)
{
  {
    auto p = path("");
    EXPECT_FALSE(p.exists());
  }
  {
    auto p = path(".");
    EXPECT_TRUE(p.exists());
  }
  {
    auto p = path("..");
    EXPECT_TRUE(p.exists());
  }
  {
    if (is_win32) {
      auto p = path("\\");
      EXPECT_TRUE(p.exists());
    } else {
      auto p = path("/");
      EXPECT_TRUE(p.exists());
    }
  }
}

/**
 * Test filesystem manipulation API.
 *
 * NOTE: expects the current directory to be write-only, else test will fail.
 *
 */
TEST(TestFilesystemHelper, filesystem_manipulation)
{
  auto dir = path(build_directory_path());
  (void)rcpputils::fs::remove(dir);
  EXPECT_FALSE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::create_directories(dir));
  EXPECT_TRUE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::is_directory(dir));
  EXPECT_FALSE(rcpputils::fs::is_regular_file(dir));

  auto file = dir / "test_file.txt";
  uint64_t expected_file_size = 0;
  {
    std::ofstream output_buffer{file.string()};
    output_buffer << "test";
    expected_file_size = static_cast<uint64_t>(output_buffer.tellp());
  }

  EXPECT_TRUE(rcpputils::fs::exists(file));
  EXPECT_TRUE(rcpputils::fs::is_regular_file(file));
  EXPECT_FALSE(rcpputils::fs::is_directory(file));
  EXPECT_GE(rcpputils::fs::file_size(file), expected_file_size);
  EXPECT_THROW(rcpputils::fs::file_size(dir), std::system_error) <<
    "file_size is only applicable for files!";
  EXPECT_FALSE(rcpputils::fs::remove(dir));
  EXPECT_TRUE(rcpputils::fs::remove(file));
  EXPECT_THROW(rcpputils::fs::file_size(file), std::system_error);
  EXPECT_TRUE(rcpputils::fs::remove(dir));
  EXPECT_FALSE(rcpputils::fs::exists(file));
  EXPECT_FALSE(rcpputils::fs::exists(dir));
  auto temp_dir = rcpputils::fs::temp_directory_path();
  temp_dir = temp_dir / "rcpputils" / "test_folder";
  EXPECT_FALSE(rcpputils::fs::exists(temp_dir));
  EXPECT_TRUE(rcpputils::fs::create_directories(temp_dir));
  EXPECT_TRUE(rcpputils::fs::exists(temp_dir));
  EXPECT_TRUE(rcpputils::fs::remove(temp_dir));
  EXPECT_TRUE(rcpputils::fs::remove(temp_dir.parent_path()));

  // Remove empty directory
  EXPECT_FALSE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::create_directories(dir));
  EXPECT_TRUE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::is_directory(dir));
  EXPECT_TRUE(rcpputils::fs::remove_all(dir));
  EXPECT_FALSE(rcpputils::fs::is_directory(dir));

  // Remove non-existing directory
  EXPECT_FALSE(rcpputils::fs::remove_all(rcpputils::fs::path("some") / "nonsense" / "dir"));

  EXPECT_FALSE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::create_directories(dir));

  // Remove single file with remove_all
  file = dir / "remove_all_single_file.txt";
  {
    std::ofstream output_buffer{file.string()};
    output_buffer << "some content";
  }
  ASSERT_TRUE(rcpputils::fs::exists(file));
  ASSERT_TRUE(rcpputils::fs::is_regular_file(file));
  EXPECT_TRUE(rcpputils::fs::remove_all(file));

  // Remove directory and its content
  EXPECT_TRUE(rcpputils::fs::exists(dir));
  EXPECT_TRUE(rcpputils::fs::is_directory(dir));

  auto num_files = 10u;
  for (auto i = 0u; i < num_files; ++i) {
    std::string file_name = std::string("test_file") + std::to_string(i) + ".txt";
    auto file = dir / file_name;
    {
      std::ofstream output_buffer{file.string()};
      output_buffer << "test" << i;
    }
  }

  // remove shall fail given that directory is not empty
  ASSERT_FALSE(rcpputils::fs::remove(dir));

  EXPECT_TRUE(rcpputils::fs::remove_all(dir));

  for (auto i = 0u; i < num_files; ++i) {
    std::string file_name = std::string("test_file") + std::to_string(i) + ".txt";
    auto file = dir / file_name;
    ASSERT_FALSE(rcpputils::fs::exists(file));
  }
  ASSERT_FALSE(rcpputils::fs::exists(dir));
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

TEST(TestFilesystemHelper, get_cwd)
{
  std::string expected_dir = rcpputils::get_env_var("EXPECTED_WORKING_DIRECTORY");
  auto p = rcpputils::fs::current_path();
  EXPECT_EQ(expected_dir, p.string());
}
