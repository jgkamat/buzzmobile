import unittest
from tests.test_utils import with_roscore, mock_pub, check_topic, with_launch_file, launch_node
import numpy as np
from collections import namedtuple
from process.bearing import calculate_directions
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

fix = namedtuple('fix', ['latitude', 'longitude'])


class TestBearing(unittest.TestCase):
    def test_distance(self):
        fix1 = fix(33.636700, -84.427863)
        fix2 = fix(39.029128, -111.838257)
        assert np.isclose(calculate_directions.get_distance(fix1, fix2), 2517000, rtol=.01)


@with_roscore
class TestBearingNode(unittest.TestCase):
    
    @with_launch_file('buzzmobile', 'test_params.launch')
    @launch_node('buzzmobile', 'bearing.py')
    def test_bearing_node(self):
        def callback(s, data):
            s.result = data.data

        with mock_pub('/fix', NavSatFix, queue_size=None) as fix_node:
            with check_topic('/buzzmobile/bearing', Float64, callback) as ct:
                # send mock data
                fix_node.send(NavSatFix(None, None, 33.636700, -84.427863, None, None, None))
                fix_node.send(NavSatFix(None, None, 39.029128, -111.838257, None, None, None))

            # check the output from the node
            yield from ct.wait_for_message()
            assert np.isclose(ct.message.data, 1.19212)
            
