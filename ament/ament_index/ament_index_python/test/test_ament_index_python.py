# Copyright 2015 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python import get_package_prefix
from ament_index_python import get_package_share_directory
from ament_index_python import get_packages_with_prefixes
from ament_index_python import get_resource
from ament_index_python import get_resources
from ament_index_python import get_search_paths
from ament_index_python import has_resource
from ament_index_python import PackageNotFoundError


def set_ament_prefix_path(subfolders):
    paths = []
    base_path = os.path.dirname(__file__)
    for subfolder in subfolders:
        path = os.path.join(base_path, subfolder)
        if os.path.isdir(path):
            paths.append(path)
    ament_prefix_path = os.pathsep.join(paths)
    os.environ['AMENT_PREFIX_PATH'] = ament_prefix_path


def test_empty_search_paths():
    set_ament_prefix_path([])
    try:
        get_search_paths()
        assert False, 'get_search_paths() failed to raise exception'
    except EnvironmentError:
        pass
    except Exception as e:
        assert False, 'get_search_paths() raised wrong exception: ' + type(e)


def test_search_paths():
    set_ament_prefix_path(['prefix1', 'prefix2'])
    search_paths = get_search_paths()
    assert len(search_paths) == 2, 'Expected two search paths'


def test_not_existing_search_paths():
    set_ament_prefix_path(['prefix1', 'not_existing_prefix'])
    search_paths = get_search_paths()
    assert len(search_paths) == 1, 'Expected one search paths'


def test_unknown_resources():
    set_ament_prefix_path(['prefix1'])
    resources = get_resources('unknown_resource_type')
    assert len(resources) == 0, 'Expected no resources'


def test_resources():
    set_ament_prefix_path(['prefix1'])
    resources = get_resources('resource_type1')
    assert len(resources) == 2, 'Expected two resources'
    assert set(resources.keys()) == {'foo', 'bar'}, 'Expected different resources'


def test_resources_overlay():
    set_ament_prefix_path(['prefix1', 'prefix2'])
    resources = get_resources('resource_type2')
    assert len(resources) == 2, 'Expected two resource'
    assert set(resources.keys()) == {'foo', 'bar'}, 'Expected different resources'


def test_resources_underlay():
    set_ament_prefix_path(['prefix1', 'prefix2'])
    resources = get_resources('resource_type3')
    assert len(resources) == 1, 'Expected one resource'
    assert set(resources.keys()) == {'bar'}, 'Expected different resources'


def test_unknown_resource():
    set_ament_prefix_path(['prefix1'])
    exists = has_resource('resource_type4', 'bar')
    assert not exists, 'Resource should not exist'

    try:
        get_resource('resource_type4', 'bar')
        assert False, 'get_resource() failed to raise exception'
    except LookupError:
        pass
    except Exception as e:
        assert False, 'get_resource() raised wrong exception: ' + type(e)


def test_resource():
    set_ament_prefix_path(['prefix1'])
    exists = has_resource('resource_type4', 'foo')
    assert exists, 'Resource should exist'

    resource, prefix = get_resource('resource_type4', 'foo')
    assert resource == 'foo', 'Expected different content'
    assert os.path.basename(prefix) == 'prefix1', 'Expected different prefix'


def test_resource_overlay():
    set_ament_prefix_path(['prefix1', 'prefix2'])

    resource, prefix = get_resource('resource_type5', 'foo')
    assert resource == 'foo1', 'Expected different content'
    assert os.path.basename(prefix) == 'prefix1', 'Expected different prefix'


def test_get_packages_with_prefixes():
    set_ament_prefix_path(['prefix1', 'prefix2'])

    packages = get_packages_with_prefixes()
    assert 'foo' in packages, "Expected to find 'foo'"
    assert os.path.basename(packages['foo']) == 'prefix1', "Expected to find 'foo' in 'prefix1'"
    assert 'bar' in packages, "Expected to find 'bar'"
    assert os.path.basename(packages['bar']) == 'prefix1', "Expected to find 'bar' in 'prefix1'"
    assert 'baz' in packages, "Expected to find 'baz'"
    assert os.path.basename(packages['baz']) == 'prefix2', "Expected to find 'baz' in 'prefix2'"

    os.environ['AMENT_PREFIX_PATH'] = '/path/does/not/exist'

    assert not get_packages_with_prefixes(), 'Expected to find no packages'


def test_get_package_prefix():
    set_ament_prefix_path(['prefix1', 'prefix2'])

    def get_package_prefix_basename(package_name):
        return os.path.basename(get_package_prefix(package_name))

    assert get_package_prefix_basename('foo') == 'prefix1', "Expected 'foo' in 'prefix1'"
    # found in both prefix1 and prefix2, but prefix1 is ahead on the APP
    assert get_package_prefix_basename('bar') == 'prefix1', "Expected 'bar' in 'prefix2'"
    assert get_package_prefix_basename('baz') == 'prefix2', "Expected 'baz' in 'prefix2'"

    try:
        get_package_prefix('does_not_exist')
    except PackageNotFoundError:
        pass
    except Exception as exc:
        assert False, 'Expected PackageNotFoundError, got: {}'.format(type(exc))

    try:
        get_package_prefix('does_not_exist')
    except KeyError:
        pass
    except Exception as exc:
        assert False, 'Expected KeyError or subclass, got: {}'.format(type(exc))


def test_get_package_share_directory():
    set_ament_prefix_path(['prefix1', 'prefix2'])

    def get_package_share_directory_test(package_name, expect_prefix):
        full_share_dir = get_package_share_directory(package_name)
        left_over, dirname = os.path.split(full_share_dir)
        assert dirname == package_name, "Expected package name '{}'".format(package_name)
        left_over, dirname = os.path.split(left_over)
        assert dirname == 'share', "Expected 'share'"
        left_over, dirname = os.path.split(left_over)
        assert dirname == expect_prefix, "Expected '{}'".format(expect_prefix)

    get_package_share_directory_test('foo', 'prefix1')
    # found in both prefix1 and prefix2, but prefix1 is ahead on the APP
    get_package_share_directory_test('bar', 'prefix1')
    get_package_share_directory_test('baz', 'prefix2')

    try:
        get_package_share_directory('does_not_exist')
    except PackageNotFoundError:
        pass
    except Exception as exc:
        assert False, 'Expected PackageNotFoundError, got: {}'.format(type(exc))
