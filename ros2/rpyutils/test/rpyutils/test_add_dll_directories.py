# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rpyutils import add_dll_directories_from_env


def test_add_dll_direcotires_from_env(monkeypatch, tmp_path):
    # Test with empty value
    monkeypatch.delenv('TEST_ENV', raising=False)
    with add_dll_directories_from_env('TEST_ENV'):
        pass

    # Test with one path
    monkeypatch.setenv('TEST_ENV', tmp_path.name)
    with add_dll_directories_from_env('TEST_ENV'):
        pass

    # Test with multiple paths
    dir1 = tmp_path / 'subdir1'
    dir2 = tmp_path / 'subdir2'
    dir1.mkdir()
    dir2.mkdir()
    monkeypatch.setenv('TEST_ENV', f'{dir1.name};{dir2.name}')
    with add_dll_directories_from_env('TEST_ENV'):
        pass
