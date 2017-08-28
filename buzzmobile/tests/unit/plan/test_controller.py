from pyrostest import RosTest, with_launch_file, launch_node
import numpy as np

from sensor_msgs.msg import Joy
from buzzmobile.msg import CarPose
from buzzmobile.msg import CarState

class TestCarState(RosTest):

    @with_launch_file('buzzmobile', 'test_params.launch')
    @launch_node('buzzmobile', 'controller')
    def test_state(self):
        with self.mock_pub('/joy', Joy, queue_size=0) as joy_node:
            with self.check_topic('/buzzmobile/car_state', CarState) as cs:
                # Ensure starts in Start Mode
                assert True
                # TODO(cole): finish writing test
