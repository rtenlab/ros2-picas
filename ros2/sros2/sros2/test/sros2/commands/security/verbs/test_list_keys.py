# Copyright 2019 Canonical Ltd
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
import tempfile

from ros2cli import cli
from sros2.api import create_key, create_keystore


def test_list_keys(capsys):
    key_names = ['/test_node', '/test_namespace/test_node', '/sky/is/the/limit']
    with tempfile.TemporaryDirectory() as keystore_dir:
        with capsys.disabled():
            # First, create the keystore
            assert create_keystore(keystore_dir)

            # Now using that keystore, create a keypair
            for key in key_names:
                assert create_key(keystore_dir, key)

        # Now verify that the key we just created is included in the list
        assert cli.main(argv=['security', 'list_keys', keystore_dir]) == 0
        assert capsys.readouterr().out.strip() == '\n'.join(sorted(key_names))


def test_list_keys_no_keys(capsys):
    with tempfile.TemporaryDirectory() as keystore_dir:
        with capsys.disabled():
            # First, create the keystore
            assert create_keystore(keystore_dir)

        # Now verify that empty keystore we just created contains no keys
        assert cli.main(argv=['security', 'list_keys', keystore_dir]) == 0
        assert len(capsys.readouterr().out.strip()) == 0


def test_list_keys_uninitialized_keystore(capsys):
    with tempfile.TemporaryDirectory() as keystore_dir:
        # Verify that list_keys properly handles an uninitialized keystore
        assert cli.main(argv=['security', 'list_keys', keystore_dir]) == 0
        assert len(capsys.readouterr().out.strip()) == 0


def test_list_keys_no_keystore(capsys):
    # Verify that list_keys properly handles a non-existent keystore
    keystore = os.path.join(tempfile.gettempdir(), 'non-existent')
    assert cli.main(argv=['security', 'list_keys', keystore]) != 0
    assert capsys.readouterr().err.strip() == 'No such file or directory: {!r}'.format(keystore)
