"""
This script runs the 16-state EKF on the test data in the test_data folder.
"""

import math
import os
import sys
from matplotlib import pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from ins_data_utilities import *
from ins_ekf_16_states import NavEKF16

def plot_ekf_estimation_error():
    tracker = NavEKF16()

    gnss_pos, imu, true_pos = read_test_data('test_data')
    roll, pitch, yaw = true_pos[1, 4:7]
    # Initialize, with first GPS data point
    tracker.initialize(gnss_pos[0, 1:4], gnss_pos[0, 4:7], np.array([yaw, pitch, roll]))

    dt = 0.01  # 100 Hz IMU data

    true_headings = []
    est_headings = []
    headings_diff = []
    est_positions = []
    for k, obs_zk in enumerate(imu, 0):
        tracker.predict(imu[k, 4:7], imu[k, 1:4], dt)
        if k % 5 == 0:
            gnss_index = k // 5
            if gnss_index < len(gnss_pos[:, 0]):
                tracker.correct(gnss_pos[gnss_index, 1:4], gnss_pos[gnss_index, 4:7], imu[gnss_index, 4:7],
                                imu[gnss_index, 1:4])

        # Log result of iteration
        # Estimated heading
        ypr = tracker.get_euler_angles()
        est_heading = ypr[0]
        est_headings.append(est_heading)
        est_positions.append(np.copy(tracker.est_pos))
        # Ground truth heading
        true_heading = np.interp(imu[k, 0], true_pos[:, 0], true_pos[:, 6])
        true_headings.append(true_heading)

        headings_diff.append(math.degrees(ang_diff_min(est_heading, true_heading)))

    # Plot results
    # Est. positions accuracy
    plt.figure(1)
    plt.scatter(gnss_pos[:, 1], gnss_pos[:, 2], label='True positions')
    estimated_pos = np.array(est_positions)
    plt.scatter(estimated_pos[:, 0], estimated_pos[:, 1], label='Estimated positions')
    plt.legend()
    plt.figure(2)
    plt.plot(imu[:, 0], headings_diff)
    plt.show()


if __name__ == '__main__':
    plot_ekf_estimation_error()
