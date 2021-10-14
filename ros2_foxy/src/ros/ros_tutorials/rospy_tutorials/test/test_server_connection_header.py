#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: add_two_ints_client 3804 2009-02-11 02:16:00Z rob_wheeler $

## Extended version of add_two_int_client that shows how to use
## connection header to pass in metadata to service.

from __future__ import print_function

PKG = 'rospy_tutorials'
NAME = 'test_server_connection_header'

import sys
import time
import unittest

from rospy_tutorials.srv import AddTwoInts, AddTwoIntsResponse
import rospy
import rostest

class TestServerConnectionHeader(unittest.TestCase):
    def __init__(self, *args):
        super(TestServerConnectionHeader, self).__init__(*args)
        self.success = False
        rospy.init_node(NAME, anonymous=True)
        s = rospy.Service('add_two_ints_header_test', AddTwoInts, self.handle_request)        
        
    def handle_request(self, req):
        if 'cookies' in req._connection_header:
            print("GOT", req._connection_header['cookies'])
            self.success = req._connection_header['cookies'] == 'peanut butter'
        return AddTwoIntsResponse(3)

    def test_header(self):
        timeout_t = time.time() + 10.0*1000 #10 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success, str(self.success))
        
if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestServerConnectionHeader, sys.argv)

