import unittest
from tests.test_utils import with_roscore, mock_pub, check_topic, with_launch_file, launch_node
import numpy as np
import polyline as pl
from collections import namedtuple
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_msgs.msg import String

fix = namedtuple('fix', ['latitude', 'longitude'])

@with_roscore
class TestGPSMapperNode(unittest.TestCase):

    @with_launch_file('buzzmobile', 'test_params.launch')
    @launch_node('buzzmobile', 'gps_mapper.py')
    def test_gps_mapper_node(self):
        fix1 = fix(33.636700, -84.427863)
        fix2 = fix(39.029128, -111.838257)
        bearing = 1.19212
        with mock_pub('/buzzmobile/bearing', Float64, queue_size=None) as bearing_node:
            with check_topic('/buzzmobile/gps_model', Image) as ct:
                bearing_node.send(Float64(bearing))
                assert ct.message() is None
            with mock_pub('/fix', NavSatFix, queue_size=None) as fix_node:
                fix_node.send(NavSatFix(None, None, 33.636700, -84.427863, None, None, None))
                fix_node.send(NavSatFix(None, None, 39.029128, -111.838257, None, None, None))
                assert ct.message() is None
                with mock_pub('/buzzmobile/polyline', String) as polyline_node:
                    polyline_node.send(String("{ucmEnwbbOjFAExDiFCBsB"))
            yield ct.wait_for_message()
                assert ct.isclose(ct.message.data, )
