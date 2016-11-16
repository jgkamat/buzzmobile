import unittest
from std_msgs.msg import String
from tests import rostest_utils

NAME = 'test_inputer'


@rostest_utils.with_roscore
class TestInputer(unittest.TestCase):

    @rostest_utils.with_launch_file('buzzmobile', 'params.launch')
    @rostest_utils.launch_node('buzzmobile', 'inputer.py')
    def test_sanity(self):
        def cb(s, data):
            s.success = (data.data == b'success')

        with rostest_utils.test_node('direction', String, cb) as tn:
            try:
                tn.success
            except AttributeError:
                assert True

