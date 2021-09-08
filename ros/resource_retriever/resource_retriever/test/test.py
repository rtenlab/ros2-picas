import resource_retriever as r

import os
import rospkg
from nose.tools import raises

rospack = rospkg.RosPack()

def test_get_by_package():
    res = r.get("package://resource_retriever/test/test.txt")
    assert len(res) == 1
    assert res == 'A'.encode()

def test_get_large_file():
    res_path = os.path.join(rospack.get_path("resource_retriever"), "test/large_filepy.dat")
    with open(res_path, 'w') as f:
        for _ in range(1024*1024*50):
            f.write('A')
    res = r.get("package://resource_retriever/test/large_filepy.dat")
    assert len(res) == 1024*1024*50

def test_http():
    res = r.get("http://packages.ros.org/ros.key")
    assert len(res) > 0

@raises(Exception)
def test_invalid_file():
    r.get("file://fail")

@raises(Exception)
def test_no_file():
    r.get("package://roscpp")

@raises(rospkg.common.ResourceNotFound)
def test_invalid_package():
    r.get("package://invalid_package_blah/test.xml")
