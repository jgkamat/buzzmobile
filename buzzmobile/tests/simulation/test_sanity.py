from tests.test_utils import RosTest, with_launch_file


class TestBearingNode(RosTest):

    @with_launch_file('buzzmobile', 'simulation.launch')
    def test_bearing_node(self):
        assert True

