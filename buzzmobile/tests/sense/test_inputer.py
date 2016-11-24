"""Tests for inputer node."""
import unittest
from std_msgs.msg import String
from tests.rostest_utils import with_roscore, check_topic, with_launch_file, launch_node


@with_roscore
class TestInputer(unittest.TestCase):

    @with_launch_file('buzzmobile', 'params.launch')
    @launch_node('buzzmobile', 'inputer.py')
    def test_sanity(self):
        def cb(s, data):
            s.success = (data.data == b'success')

        with check_topic('direction', String, cb) as ct:
            try:
                ct.success
            except AttributeError:
                assert True
