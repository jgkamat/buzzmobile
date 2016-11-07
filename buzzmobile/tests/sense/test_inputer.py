import unittest
import rospy
import time
from std_msgs.msg import String
from tests import rostest_utils

NAME = 'test_inputer'


@rostest_utils.with_roscore
class TestInputer(unittest.TestCase):

    def run_node_with_callback(self, callback):
        rospy.init_node(NAME, anonymous=True)

        rospy.Subscriber('destination', String, callback)
        timeout = time.time() + 2.0

        while time.time() < timeout and not rospy.is_shutdown():
            time.sleep(0.1)

    def __init__(self, *args):
        super().__init__(*args)

    def setUp(self):
        self.success = False

    @rostest_utils.launch_node('buzzmobile', 'inputer.py')
    def test_inputer_no_input(self):
        def cb(data):
            self.success = (data.data == '')

        self.run_node_with_callback(cb)
        assert self.success

    @rostest_utils.launch_node('buzzmobile', 'inputer.py')
    def test_sanity(self):
        def cb(data):
            self.success = (data.data == b'success')

        self.run_node_with_callback(cb)
        assert not self.success

