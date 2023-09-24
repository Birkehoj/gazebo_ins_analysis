from unittest import TestCase
import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from ins_data_utilities import read_test_data


class TestReadFile(TestCase):
    def test_read_file(self):
        gnss, imu, true_poses = read_test_data('test_data')
        self.assertTrue(np.array_equal(gnss[0, :], np.array([0.001, 0.0311018295, -0.0855829599, -9.99681652e-06],
                                                            dtype=np.float64)))
        self.assertTrue(np.array_equal(imu[0, :], np.array(
            [0.001, -6.4905228e-19, -1.76145952e-18, 6.62963326e-19, -8.84659274e-16, -2.18696339e-16, 0],
            dtype=np.float64)))
        self.assertTrue(np.array_equal(true_poses[0, :], np.array(
            [0.05, -5.40674593e-05, -2.61227934e-07, -0.0023148171, 4.02481424e-05, 0.000265064227, -1.41848533e-05,
             0.00359680289], dtype=np.float64)))
        self.assertEqual(len(gnss), 5)
        self.assertEqual(len(imu), 5)
        self.assertEqual(len(true_poses), 5)
