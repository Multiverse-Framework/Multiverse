import unittest
from multiverse_socket.multiverse_ros_base.multiverse_ros_base import SimulationMetaData


class MetadataTestCase(unittest.TestCase):
    metadata: SimulationMetaData

    @classmethod
    def setUpClass(cls):
        cls.metadata = SimulationMetaData()

    def test_to_json(self):
        print(self.metadata.__dict__)


if __name__ == "__main__":
    unittest.main()
