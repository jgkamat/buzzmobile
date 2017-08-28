from pyrostest import RosTest, with_launch_file
from rosgraph_msgs.msg import Clock
import time

class TestGazeboConnection(RosTest):
    """Tests for simulation testing infrastructure."""

    @with_launch_file('buzzmobile', 'simulation.launch')
    def test_clock_runs(self):
        """Test that simulation.launch correctly launches gazebo"""
        with self.check_topic('/clock', Clock) as ct:
            assert(ct.message)
            print(ct.message)
        time.sleep(30)
        with self.check_topic('/clock', Clock, 20) as ct1:
            print(ct1.message)
            assert(ct1.message)
