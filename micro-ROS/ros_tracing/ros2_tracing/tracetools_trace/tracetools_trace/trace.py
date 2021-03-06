#!/usr/bin/env python3
# Copyright 2019 Robert Bosch GmbH
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

"""Entrypoint/script to setup and start an LTTng tracing session."""

from typing import List

from tracetools_trace.tools import args
from tracetools_trace.tools import lttng
from tracetools_trace.tools import path
from tracetools_trace.tools import print_events_list


def init(
    session_name: str,
    base_path: str,
    ros_events: List[str],
    kernel_events: List[str],
    display_list: bool = False,
) -> None:
    """
    Init and start tracing.

    :param session_name: the name of the session
    :param base_path: the path to the directory in which to create the tracing session directory
    :param ros_events: list of ROS events to enable
    :param kernel_events: list of kernel events to enable
    :param display_list: whether to display list(s) of enabled events
    """
    ust_enabled = len(ros_events) > 0
    kernel_enabled = len(kernel_events) > 0
    if ust_enabled:
        print(f'UST tracing enabled ({len(ros_events)} events)')
        if display_list:
            print_events_list(ros_events)
    else:
        print('UST tracing disabled')
    if kernel_enabled:
        print(f'kernel tracing enabled ({len(kernel_events)} events)')
        if display_list:
            print_events_list(kernel_events)
    else:
        print('kernel tracing disabled')

    full_session_path = path.get_full_session_path(session_name, base_path)
    print(f'writing tracing session to: {full_session_path}')
    input('press enter to start...')
    lttng.lttng_init(
        session_name,
        base_path=base_path,
        ros_events=ros_events,
        kernel_events=kernel_events)


def fini(
    session_name: str,
) -> None:
    """
    Stop and finalize tracing.

    :param session_name: the name of the session
    """
    input('press enter to stop...')
    print('stopping & destroying tracing session')
    lttng.lttng_fini(session_name)


def main():
    params = args.parse_args()

    init(
        params.session_name,
        params.path,
        params.events_ust,
        params.events_kernel,
        params.list,
    )
    fini(
        params.session_name,
    )
