import unittest
from tests import rostest_utils
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


@rostest_utils.with_roscore
class TestBearingNode(unittest.TestCase):
    
    @rostest_utils.with_launch_file('buzzmobile', 'params.launch')
    @rostest_utils.launch_node('buzzmobile', 'bearing.py')
    def test_bearing_node(self):
        def callback(s, data):
            s.result = data.data

        with rostest_utils.mock_node('/buzzmobile/fix', NavSatFix, queue_size=None) as fix_node:
            with rostest_utils.test_node('/buzzmobile/bearing', Float64, callback) as tn:
                # send mock data
                fix_node.send(NavSatFix(None, None, 33.636700, -84.427863, None, None, None))
                fix_node.send(NavSatFix(None, None, 39.029128, -111.838257, None, None, None))

            # check the output from the node
            assert tn.result != None
            assert np.isclose(tn.result, 5.09105)
            
