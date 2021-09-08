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

# This is a utility program to help generate consistent results from Connext rtiddsgen{_server}.
# We've seen cases where it can generate header files that do not contain the proper 'extern "C"'
# markings, which we believe is a race inside the generator itself.
# To combat this, we try up to 10 times to generate, and ensure that the header files have the
# correct markings.
# Once they do, we consider it a success.

import argparse
import os
import shutil
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--idl-pp', required=True)
    parser.add_argument('--idl-file', required=True)
    parser.add_argument('-d', required=True)
    parser.add_argument('--max-tries', type=int, default=10)
    args = parser.parse_args()

    count = 1
    while count <= args.max_tries:
        # Remove and recreate the output target directory.  This ensures that previous failed
        # attempts won't cause the next attempt to fail.
        try:
            shutil.rmtree(args.d)
        except FileNotFoundError:
            pass
        os.mkdir(args.d)

        cmdline = [
            args.idl_pp, '-language', 'C++', '-unboundedSupport', args.idl_file, '-d', args.d
        ]
        print('Running command: %s' % (' '.join(cmdline)))
        ret = subprocess.run(args=cmdline)
        if ret.returncode == 0:
            with open(os.path.join(args.d, args.idl_file[:-4] + 'Plugin.h'), 'r') as infp:
                for line in infp:
                    if line.startswith('extern "C" {'):
                        return 0

        print(f'Try {count} of {args.max_tries} failed to generate header with \'extern "C"\'',
              end='')
        count += 1
        if count <= args.max_tries:
            print(', trying again')
        else:
            print(', giving up')

    sys.stdout.flush()
    print('Could not successfully generate Connext serialized data', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
