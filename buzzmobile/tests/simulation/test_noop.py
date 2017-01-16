import unittest

# TODO(iRapha): Remove this test when we have actual simulation tests.
class TestNoop(unittest.TestCase):
    """Class showing proof of concept of simulation/unit test split.
    """

    def setUp(self):
        """Sets up test environment.
        """
        self.true = True

    def test_example(self):
        """Example test method.
        """
        assert self.true
