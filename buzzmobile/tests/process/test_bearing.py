import unittest
from tests import rostest_utils
import numpy as np
from collections import namedtuple
from process.bearing import calculate_directions

fix = namedtuple('fix', ['latitude', 'longitude'])


class TestBearing(unittest.TestCase):
    def test_distance(self):
        fix1 = fix(33.636700, -84.427863)
        fix2 = fix(39.029128, -111.838257)
        assert np.isclose(calculate_directions.get_distance(fix1, fix2), 2517000, rtol=.01)


@rostest_utils.with_roscore
class TestBearingNode(unittest.TestCase):
    pass

