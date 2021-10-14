# Copyright 2016 Open Source Robotics Foundation, Inc.
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

# Based on `unifieddiff.py` by @noporpoise which is licensed as `Public domain (CC0)`
# see: https://gist.github.com/noporpoise/16e731849eb1231e86d78f9dfeca3abc
# TODO(Karsten1987): Rewrite this function for clarity and tests

import argparse
import re
import sys

# regular expression / pattern for patch header
_hdr_pat = re.compile(r'^@@ -(\d+),?(\d+)? \+(\d+),?(\d+)? @@.*$')


def apply_patch(s, patch):
    """
    Apply unified diff patch to string s to recover newer string.

    If revert is True, treat s as the newer string, recover older string.
    """
    s = s.splitlines(True)
    p = patch.splitlines(True)
    t = ''
    i = sl = 0
    (midx, sign) = (1, '+')
    while i < len(p) and p[i].startswith(('---', '+++')):
        i += 1  # skip header lines
    while i < len(p):
        # find patch header
        m = _hdr_pat.match(p[i])
        if not m:
            raise Exception('Cannot process diff in line ' + str(i))

        i += 1
        ll = int(m.group(midx)) - 1 + (m.group(midx + 1) == '0')
        t += ''.join(s[sl:ll])
        sl = ll
        while i < len(p) and p[i][0] != '@':
            if i + 1 < len(p) and p[i + 1][0] == '\\':
                line = p[i][:-1]
                i += 2
            else:
                line = p[i]
                i += 1
            if len(line) > 0:
                if line[0] == ' ':
                    # ensure that any unmodified line is the same
                    # in the input file and in the patch file
                    assert s[sl] == line[1:], \
                        "s[%d] '%s' != line[1:] '%s'" % (sl, s[sl], line[1:])
                if line[0] == sign or line[0] == ' ':
                    t += line[1:]
                sl += (line[0] != sign)
    t += ''.join(s[sl:])
    return t


parser = argparse.ArgumentParser()
parser.add_argument('--input', nargs='+')
parser.add_argument('--patch', nargs='+')
parser.add_argument('--out', nargs='+')
args = parser.parse_args()
for i, p, o in zip(args.input, args.patch, args.out):
    with open(i, 'r') as h:
        content_in = h.read()
    with open(p, 'r') as h:
        content_patch = h.read()
    try:
        content_out = apply_patch(content_in, content_patch)
    except Exception:
        print(
            f'Failed to generate "{o}" using input "{i}" and patch "{p}"',
            file=sys.stderr,
        )
        print('---------input file---------', file=sys.stderr)
        print(content_in, file=sys.stderr)
        print('----------------------------', file=sys.stderr)
        print('---------patch file---------', file=sys.stderr)
        print(content_patch, file=sys.stderr)
        print('----------------------------', file=sys.stderr)
        raise
    with open(o, 'w') as h:
        h.write(content_out)
