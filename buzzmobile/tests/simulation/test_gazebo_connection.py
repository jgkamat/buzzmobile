from tests.test_utils import RosTest, with_launch_file, check_topic
from rosgraph_msgs.msg import Clock

class TestGazeboConnection(RosTest):
    """Tests for simulation testing infrastructure."""

    @with_launch_file('buzzmobile', 'simulation.launch')
    def test_clock_runs(self):
        """Test that simulation.launch correctly launches gazebo"""
        with check_topic('/clock', Clock) as ct:
            assert(ct.message)
        with check_topic('/clock', Clock) as ct:
            assert(ct.message)

