# Copyright 2019 Apex.AI, Inc.
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

import socket

# To coordinate ROS_DOMAIN_IDs between multiple test process instances, we
# open a high numbered port "PORT_BASE + ROS_DOMAIN_ID."
# If we manage to open the port, then we can use that ROS_DOMAIN_ID for the duration of the
# test run
_PORT_BASE = 22119  # I picked this randomly as a high port that probably won't be in use


class _sockwrapper():
    """Wraps sockets to keep them open, but appear like a number from 1 to 100."""

    def __init__(self, socket):
        self.__socket = socket

    def __str__(self):
        return str(self.__socket.getsockname()[1] - _PORT_BASE)


class _default_selector:

    def __init__(self):
        # When we need to coordinate 10 or 20 domains, it's about 10x faster
        # to start with a random seed value here.
        # It's also the difference between 1ms and 100us so it's totally insignificant.
        # Leaving this here in case it's ever useful in the future to speed up domain selection:
        # self._value = random.randint(1, 100)

        # Slower, but deterministic:
        # Always start at '1' so if there's weirdness where domains are colliding when
        # they shouldn't, it's easier to debug.
        self._value = 1

    def __call__(self):
        retval = ((self._value - 1) % 100) + 1
        self._value += 1
        return retval


def get_coordinated_domain_id(*, selector=None):
    """
    Get a ROS_DOMAIN_ID from 1 to 100 that will not conflict with other ROS_DOMAIN_IDs.

    Processes can use get_coordinated_domain_id to generate ROS_DOMAIN_IDs that allow them to
    use ROS2 without unexpected cross-talk between processes.
    This is similar to the ROS1 rostest behavior of putting the ROS master on a unique port.

    Users of get_coordintaed_gomain_id must keep the returned object alive.  If the returned
    object is garbage collected, the ROS_DOMAIN_ID it represents is returned to the pool
    of available values.
    """
    if selector is None:
        selector = _default_selector()

    # Try 100 times to get a unique ROS domain ID.  The default number of parallel colcon
    # test runners is 12, so it's extremely unlikely that more than 12 ROS_DOMAIN_IDs need
    # to be coordinated at once.
    for attempt in range(100):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind(('', _PORT_BASE + selector()))
        except OSError:
            continue
        else:
            return _sockwrapper(s)
    else:
        raise Exception('Failed to get a unique domain ID')
