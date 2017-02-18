import unittest
import time
from tests.test_utils import RosTest, with_launch_file

# TODO(iRapha): Remove this test when we have actual simulation tests.
class TestNoop(RosTest):
    """Class showing proof of concept of simulation/unit test split.
    """

    def setUp(self):
        """Sets up test environment.
        """
        self.true = True

    # TODO(irapha): rever this file before committing.
    # Only do so when you can sigkill this test
    @with_launch_file('buzzmobile', 'simulation.launch')
    def test_example(self):
        """Example test method.
        """
        time.sleep(30)
        assert self.true
