import unittest
import numpy as np
from models.quaternion import Quaternion


class TestQuaternion(unittest.TestCase):
    def setUp(self) -> None:
        self.q = Quaternion()

    def test_create_q_blank(self):
        q = Quaternion()
        pure_vector = np.array([1, 0, 0, 0], dtype=np.float64)
        self.assertTrue(np.array_equal(q.get_q(), pure_vector))

    def test_create_q_with_q(self):
        self.q = Quaternion(np.array([0.7071067, 0, 0, 0.7071067]), auto_normalize=True)

        q = np.array([0.70710678, 0, 0, 0.70710678], dtype=np.float64)
        # euler = np.array([0, 0, 90], dtype=np.float64)
        # rotation_vector = np.array([90, 0, 0, 1], dtype=np.float64)
        # dcm = np.array([
        #     [0, -1, 0],
        #     [1, 0, 0],
        #     [0, 0, 1]
        # ])
        print(self.q.get_q())
        self.assertTrue(np.array_equal(self.q.get_q(), q))
        # self.assertTrue(np.array_equal(np.around(self.q.get_euler()), euler))
        # self.assertTrue(np.array_equal(self.q.get_rotation_vector(), rotation_vector))
        # self.assertTrue(np.array_equal(self.q.get_dcm(), dcm))

    # def test_create_q_with_euler(self):
    #     self.q = Quaternion()
    #     self.q.make_from_euler(euler=np.array([45, 45, 0]))
    #
    #     print(self.q.get_q())
        # q = np.array([0.6532814824381883, 0.27059805007309845, 0.2705980500730985, 0.6532814824381884], dtype=np.float64)
        # euler = np.array([45, 45, 0], dtype=np.float64)
        # rotation_vector = np.array([90, 1, 1, 0], dtype=np.float64)
        # dcm = np.array([
        #     [0, -1, 0],
        #     [1, 0, 0],
        #     [0, 0, 1]
        # ])

        # self.assertTrue(np.array_equal(self.q.get_q(), q))
        # self.assertTrue(np.array_equal(np.around(self.q.get_euler()), euler))
        # self.assertTrue(np.array_equal(self.q.get_rotation_vector(), rotation_vector))
        # self.assertTrue(np.array_equal(self.q.get_dcm(), dcm))




if __name__ == "__main__":
    unittest.main()
