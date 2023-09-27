from filterpy.kalman import KalmanFilter
from matplotlib import pyplot as plt
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
import numpy as np
import math
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from ins_data_utilities import *

def main():
    # Constant velocity filter
    tracker = KalmanFilter(dim_x=4, dim_z=4)
    dt = 1 / 20  # time step 1 second
    R_std = 0.02 / 2  # 3D CQ value divided with 2 because only two dimensions are considered
    Q_std = 0.025  # How precise is constant velocity assumption? Don't know, probably just tuning parameter
    tracker.F = np.array([[1, dt, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, dt],
                          [0, 0, 0, 1]])

    q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_std ** 2)
    tracker.Q = block_diag(q, q)
    tracker.H = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]
                          ])
    tracker.R = np.eye(4) * R_std ** 2
    tracker.R[1, 1] = 0  # trust totally on velocity measurements
    tracker.R[3, 3] = 0  # Does not work, TODO: figure out proper noise on gnss velocity
    tracker.P = np.eye(4) * 1 ** 2  # initial uncertainty

    # Adaptive filter params
    Q_scale_factor = 3
    std_scale = 1

    gnss_pos, _, true_pos = read_test_data('test_data')

    true_times = true_pos[:, 0]
    true_headings = true_pos[:, 6]

    # Initialize filter with first position
    tracker.x = np.array([[true_pos[0, 0], 0, true_pos[0, 1], 0]]).T  # initial state

    estimated_pos = []
    headings = []
    headings_diff = []
    data_heading = []
    phi = Q_std
    count = 0
    for k, obs_zk in enumerate(gnss_pos, 0):
        prev_x = tracker.x.flatten()
        if not k == 0:  # initial measurement
            tracker.predict()
            tracker.update([[obs_zk[1]], [gnss_pos[k, 4]], [obs_zk[2]], [gnss_pos[k, 5]]])
        estimated_pos.append(tracker.x.flatten())

        std = math.sqrt(tracker.S[0, 0])
        # print(tracker.y, std)
        if abs(tracker.y[0, 0]) > std_scale * std:
            phi *= Q_scale_factor
            q = Q_discrete_white_noise(dim=2, dt=dt, var=phi ** 2)
            tracker.Q = block_diag(q, q)
            count += 1
        elif count > 0:
            phi /= Q_scale_factor
            q = Q_discrete_white_noise(dim=2, dt=dt, var=phi ** 2)
            tracker.Q = block_diag(q, q)
            count -= 1
        print(tracker.Q)
        print(phi)
        direction = tracker.x.flatten() - prev_x
        heading = math.atan2(direction[2], direction[0])
        headings.append(heading)
        true_heading = np.interp(obs_zk[0], true_times, true_headings)
        data_heading.append(true_heading)
        headings_diff.append(math.degrees(ang_diff_min(heading, true_heading)))

    plt.figure(1)
    plt.scatter(true_pos[:, 1], true_pos[:, 2], label='True positions')
    plt.scatter(np.array(estimated_pos)[:, 0], np.array(estimated_pos)[:, 2],label='Estimated positions')
    plt.legend()
    plt.figure(2)
    plt.plot(gnss_pos[:,0], headings_diff)

    plt.figure(3)
    plt.plot( gnss_pos[:,4], label='GNSS speed')
    plt.show()



if __name__ == '__main__':
    main()
