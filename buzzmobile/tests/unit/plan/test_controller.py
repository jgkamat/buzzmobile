import unittest
from tests.test_utils import with_roscore, mock_pub, check_topic, with_launch_file, launch_node, await
import numpy as np
from collections import namedtuple

from sensor_msgs.msg import Joy
from buzzmobile.msg import CarPose
from buzzmobile.msg import CarState

@with_roscore
class TestCarState(unittest.TestCase):

    @with_launch_file('buzzmobile', 'test_params.launch')
    @launch_node('buzzmobile', 'controller')
    def test_state(self):
         with mock_pub('/joy', Joy, queue_size=None) as joy_node:

            # TODO When this line is enabled, it causes the bearing test to
            # fail stating the message has not been sent yet.
            # Even though there is an await in the bearing test.
            # Also, it doesn't matter which topic is checked
            with check_topic('/buzzmobile/car_state', CarState) as cs:
                # Ensure starts in Start Mode
                assert True
                # TODO(cole): finish writing test
                # joy_node.send(Joy())
                # await(cs.wait_for_message())
                # assert cs.message.state == CarState.START
