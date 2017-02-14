import unittest
from tests.test_utils import with_roscore, mock_pub, check_topic, with_launch_file, launch_node, await
import numpy as np
from collections import namedtuple

from sensor_msgs.msg import Joy
from buzzmobile.msg import CarPose
from buzzmobile.msg import CarState

import time

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
               
                # Note: You must specify arrays for axes and buttons for the
                # Joy constructor with length at least 16. Otherwise it
                # does not work. My guess is the controller node has an 
                # index out of bounds exception but I haven not confirmed
                axes = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
                buttonsEmpty = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

                expectedOutcomes = [1]

                # Ensure starts in Start Mode
                joy_node.send(Joy(None, axes, buttonsEmpty))
                await(cs.wait_for_message())
                assert cs.message.state == CarState.START
                
                #for val in expectedOutcomes:
                joy_node.send(Joy(None, axes, buttons))
                joy_node.send(Joy(None, axes, buttons))
                joy_node.send(Joy(None, axes, buttonsEmpty))
                #await(joy_node.wait_for_message())
                #await(joy_node.wait_for_message())
                await(cs.wait_for_message())
                time.sleep(10)
                assert cs.message.state == 1
               
