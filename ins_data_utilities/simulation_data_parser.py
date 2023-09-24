import csv
import os
import numpy as np


def read_file(file_name):
    with open(file_name) as csv_file:
        csv_reader = csv.reader(csv_file)
        csv_reader.__next__()
        return np.array([line for line in csv_reader], dtype=np.float64)


def read_test_data(directory):
    gnss = read_file(os.path.join(directory, 'gnss.csv'))
    imu = read_file(os.path.join(directory, 'imu.csv'))
    true_pose = read_file(os.path.join(directory, 'true_pose_speed.csv'))
    return gnss, imu, true_pose
