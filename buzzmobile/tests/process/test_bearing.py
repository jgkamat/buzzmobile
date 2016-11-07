import unittest
from tests import rostest_utils
import rospy
import numpy as np
from collections import namedtuple

# we need this set before importing bearing
rospy.set_param('median_filter_size', 5)
rospy.set_param('min_fix_distance', .01)
from process.bearing import bearing

fix = namedtuple('fix', ['latitude', 'longitude'])


class TestBearing(unittest.TestCase):
    def test_distance(self):
        fix1 = fix(33.636700, -84.427863)
        fix2 = fix(39.029128, -111.838257)
        assert np.isclose(bearing.get_distance(fix1, fix2), 2517000, rtol=.01)


@rostest_utils.with_roscore
class TestBearingNode(unittest.TestCase):
    pass

