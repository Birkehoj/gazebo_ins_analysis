"""
This script runs the 16-state EKF on the test data in the test_data folder.
"""

import math
import os
import sys
from matplotlib import pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))
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
    plt.scatter(true_pos[:, 1], true_pos[:, 2], label='True positions')
    plt.scatter(gnss_pos[:, 1], gnss_pos[:, 2], label='Measured positions')
    estimated_pos = np.array(est_positions)
    plt.scatter(estimated_pos[:, 0], estimated_pos[:, 1], label='Estimated positions')
    plt.legend()
    plt.figure(2)
    # plt.plot(imu[:, 0], headings_diff)
    # plt.plot(imu[:, 0], headings_diff)
    plt.plot(imu[:, 0], true_headings, label='True heading')
    plt.plot(imu[:, 0], est_headings, label='Estimated heading')
    plt.legend()
    plt.show()

    # Print final kalman filter states
    print(f"Pos: Est. {tracker.est_pos}, Actual: {true_pos[-1, 1:4]}, Difference: {true_pos[-1, 1:4] - tracker.est_pos}")
    print(f"Vel: Est. {tracker.est_vel}, Actual: {true_pos[-1, 7:10]}, Difference: {true_pos[-1, 7:10] - tracker.est_vel}")
    print(f"Orientation: Est. {np.degrees(tracker.get_euler_angles())}, Actual: {np.degrees(true_pos[-1, 4:7])}, Difference: {np.degrees(true_pos[-1, 4:7] - tracker.get_euler_angles())}")
    print(f"acc. bias: {tracker.est_acc_bias}")
    print(f"rot. bias: {tracker.est_rate_of_rot_bias}")
    # TODO: Why is velocity in x dir not approximately 1 m/s
    # TODO: Why are the bias values so large?
    # TODO: why is the velocity bias -1.85 m/s? it is not set to be bigger than the two others



if __name__ == '__main__':
    plot_ekf_estimation_error()
