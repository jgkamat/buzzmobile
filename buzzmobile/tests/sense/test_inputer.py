import unittest
from std_msgs.msg import String
from tests import rostest_utils

NAME = 'test_inputer'


@rostest_utils.with_roscore
class TestInputer(unittest.TestCase):

    def setUp(self):
        self.success = False

    @rostest_utils.launch_node('buzzmobile', 'inputer.py', 'params.launch')
    def test_sanity(self):
        def cb(data):
            self.success = (data.data == b'success')

        with rostest_utils.test_node('direction', String, cb):
            assert not self.success

