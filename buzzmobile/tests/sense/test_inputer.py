"""Tests for inputer node."""
import rostest
import unittest
import rospy
import time
from std_msgs.msg import String
import roslaunch

PKG = 'buzzmobile'
NAME = 'test_inputer'

def run_node_with_callback(callback):
    """Initializes ros node, with callback."""
    rospy.init_node(NAME, anonymous=True)

    rospy.Subscriber('destination', String, callback)
    timeout = time.time() + 2.0

    while time.time() < timeout and not rospy.is_shutdown():
        time.sleep(0.1)

class TestInputer(unittest.TestCase):
    """Test inputer node."""

    def __init__(self, *args):
        super(TestInputer, self).__init__(*args)


    def setUp(self):
        self.success = False
        node = roslaunch.core.Node('buzzmobile', 'inputer.py')
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        self.process = launch.launch(node)

    def test_inputer_no_input(self):
        """Test no input to inputer node."""
        def cb(data):
            """callback."""
            self.success = (data.data == '')

        run_node_with_callback(cb)
        assert self.success

    def test_sanity(self):
        """Test sanity."""
        def cb(data):
            """callback."""
            self.success = (data.data == b'success')

        run_node_with_callback(cb)
        assert not self.success

    def tearDown(self):
        self.process.stop()


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestInputer)


