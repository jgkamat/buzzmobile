import rostest
import unittest
import rospy
import time
from std_msgs.msg import String
import roslaunch

PKG = 'buzzmobile'
NAME = 'test_inputer'


class TestInputer(unittest.TestCase):

    def run_node_with_callback(self, callback):
        rospy.init_node(NAME, anonymous=True)

        rospy.Subscriber('destination', String, callback)
        timeout = time.time() + 1.0

        while time.time() < timeout:
            time.sleep(0.1)

    def __init__(self, *args):
        super().__init__(*args)


    def setUp(self):
        self.success = False
        node = roslaunch.core.Node('buzzmobile', 'inputer.py')
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        self.process = launch.launch(node)

    def test_inputer_no_input(self):
        def cb(data):
            self.success = (data.data == '')

        self.run_node_with_callback(cb)
        assert self.success

    def test_sanity(self):
        def cb(data):
            self.success = (data.data == b'success')

        self.run_node_with_callback(cb)
        assert not self.success

    def tearDown(self):
        self.process.stop()


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestInputer)


