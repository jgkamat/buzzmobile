from pyrostest import RosTest, with_launch_file
from rosgraph_msgs.msg import Clock

class TestGazeboConnection(RosTest):
    """Tests for simulation testing infrastructure."""

    @with_launch_file('buzzmobile', 'simulation.launch')
    def test_clock_runs(self):
        """Test that simulation.launch correctly launches gazebo"""
        with self.check_topic('/clock', Clock) as ct:
            assert(ct.message)
        with self.check_topic('/clock', Clock) as ct:
            assert(ct.message)

