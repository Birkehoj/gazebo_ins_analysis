"""
This file contains the 16-state EKF for INS/GNSS integration in direct configuration
The following states are estimated in three dimensions
position, velocity, orientation, accel bias, gyro bias

References
https://github.com/NorthStarUAS/insgnss_tools/blob/main/insgnss_tools/Kinematics.py
https://github.com/PX4/PX4-ECL/tree/master/EKF/python/ekf_derivation
https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3812568/
"""

import math

import numpy as np

from ins_data_utilities.quaternion_utils import *


def create_skew(w):
    """
    Create skew symmetric matrix from vector
    :param w:
    :return:
    """
    return np.array([
        [0.0, -w[2], w[1]],
        [w[2], 0.0, -w[0]],
        [-w[1], w[0], 0.0]])


class NavEKF16:
    """
    16-state EKF for INS/GNSS integration in direct configuration
    """
    gravity_const = 9.81  # Acceleration due to gravity

    _I2 = np.identity(2)
    _I3 = np.identity(3)
    _I5 = np.identity(5)
    _I16 = np.identity(16)

    def __init__(self):
        # Sensor variances (as standard deviation) and models (tau)
        self.aNoiseSigma_mps2 = 0.05  # Std dev of Accelerometer Wide Band Noise (m/s^2)
        self.aMarkovSigma_mps2 = 0.1  # Std dev of Accelerometer Markov Bias
        self.tau_acc_bias = 300.0  # Correlation time or time constant

        self.wNoiseSigma_rps = math.radians(0.003393695767766752)  # Std dev of rotation rate output noise (rad/s)
        self.wMarkovSigma_rps = math.radians(0.015)  # Std dev of correlated rotation rate bias (rad/s)
        self.tau_angular_acc_bias = 1000.0  # Correlation time or time constant

        # Seems to velocity is updated wrongly
        self.rNoiseSigma_NE_m = 0.02  # GPS measurement noise std dev (m)
        self.rNoiseSigma_U_m = 0.05  # GPS measurement noise std dev (m)

        self.vNoiseSigma_NE_mps = 0.5  # GPS measurement noise std dev (m/s)
        self.vNoiseSigma_U_mps = 1.0  # GPS measurement noise std dev (m/s)

        # Initial set of covariance
        self.pErrSigma_Init_m = 1.0  # Std dev of initial position error (m)
        self.vErrSigma_Init_mps = 1.0  # Std dev of initial velocity error (m/s)
        self.attErrSigma_Init_rad = math.radians(20)  # Std dev of initial attitude (phi and theta) error (rad)
        self.hdgErrSigma_Init_rad = math.radians(90)  # Std dev of initial Heading (psi) error (rad)
        self.aBiasSigma_Init_mps2 = 0.1 * self.gravity_const  # Std dev of initial acceleration bias (m/s^2)
        self.wBiasSigma_Init_rps = math.radians(1)  # Std dev of initial rotation rate bias (rad/s)

        # Kalman Matrices
        self.H = np.zeros((6, 16))  # Observation matrix
        self.R = np.zeros((6, 6))  # Covariance of the Observation Noise (associated with MeasUpdate())
        self.Rw = np.zeros((6, 6))  # Covariance of the Sensor Noise (associated with TimeUpdate())
        self.S = np.zeros((6, 6))  # Innovation covariance
        self.P = np.zeros((16, 16))  # Covariance estimate
        self.K = np.zeros((16, 6))  # Kalman gain
        self.Q = np.zeros((16, 16))  # Process noise covariance
        self.Fx = self._I16  # State transition matrix

        # State estimates
        self.est_pos = np.zeros(3)  # Estimated position in ENU
        self.est_vel = np.zeros(3)  # Estimated velocity in ENU
        self.est_ori = np.zeros(4)  # Quaternion of B wrt Local
        self.est_acc_bias = np.zeros(3)  # acceleration bias
        self.est_rate_of_rot_bias = np.zeros(3)  # rotation rate bias

        # Estimates from previous data
        self.est_acc = np.zeros(3)  # Estimated acceleration in Body
        self.est_rate_rotation = np.zeros(3)  # Estimated rate of rotation

        self._configure()

    def _configure(self):
        # Observation matrix (H)
        self.H[0:5, 0:5] = self._I5

        # Covariance of the Process Noise (associated with TimeUpdate())
        self.Rw[0:3, 0:3] = self.aNoiseSigma_mps2 ** 2 * self._I3
        self.Rw[3:6, 3:6] = self.wNoiseSigma_rps ** 2 * self._I3
        # self.Rw[6:9, 6:9] = (2 / self.tau_acc_bias * self.aMarkovSigma_mps2 ** 2) * self._I3  # TODO: Check this
        # self.Rw[9:12, 9:12] = (2 / self.tau_angular_acc_bias * self.wMarkovSigma_rps ** 2) * self._I3

        # Covariance of the Observation Noise (associated with MeasUpdate())
        self.R[0:2, 0:2] = self.rNoiseSigma_NE_m ** 2 * self._I2
        self.R[2, 2] = self.rNoiseSigma_U_m ** 2
        self.R[3:5, 3:5] = self.vNoiseSigma_NE_mps ** 2 * self._I2
        self.R[5, 5] = self.vNoiseSigma_U_mps ** 2

        # Initial Innovation Covariance Estimate (S)
        # Zeros

        # Initial Covariance Estimate (P)
        self.P[0:3, 0:3] = self.pErrSigma_Init_m ** 2 * self._I3
        self.P[3:6, 3:6] = self.vErrSigma_Init_mps ** 2 * self._I3
        self.P[6:8, 6:8] = self.attErrSigma_Init_rad ** 2 * self._I2
        self.P[8, 8] = self.hdgErrSigma_Init_rad ** 2
        self.P[9:12, 9:12] = self.aBiasSigma_Init_mps2 ** 2 * self._I3
        self.P[12:15, 12:15] = self.wBiasSigma_Init_rps ** 2 * self._I3

    def get_euler_angles(self):
        # Euler angles from quaternion
        return quaternion_to_euler(self.est_ori)

    def initialize(self, r0, v0, s0):
        # Initialize Position and Velocity
        self.est_pos = r0  # Position in ENU
        self.est_vel = v0  # Velocity in ENU
        # Euler to quaternion
        self.est_ori = euler_to_quaternion(s0)

    def create_state_transition_matrix(self, dt) -> np.ndarray:
        q0, q1, q2, q3 = self.est_ori
        dvx, dvy, dvz = self.est_acc  # delta velocity
        dax, day, daz = self.est_rate_rotation  # delta angle
        dvx_b, dvy_b, dvz_b = self.est_acc_bias  # Accelerometer bias
        dax_b, day_b, daz_b = self.est_rate_of_rot_bias
        g = self.gravity_const
        # tau_dvb = self.tau_acc_bias
        # tau_dab = self.tau_angular_acc_bias
        return np.array(
            [[1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 1, 0, 0, dt * (2 * q2 * (dvz - dvz_b - g) - 2 * q3 * (dvy - dvy_b)),
              dt * (2 * q2 * (dvy - dvy_b) + 2 * q3 * (dvz - dvz_b - g)),
              dt * (2 * q0 * (dvz - dvz_b - g) + 2 * q1 * (dvy - dvy_b) - 4 * q2 * (dvx - dvx_b)),
              dt * (-2 * q0 * (dvy - dvy_b) + 2 * q1 * (dvz - dvz_b - g) - 4 * q3 * (dvx - dvx_b)), 0, 0, 0,
              dt * (2 * q2 ** 2 + 2 * q3 ** 2 - 1), dt * (2 * q0 * q3 - 2 * q1 * q2),
              dt * (-2 * q0 * q2 - 2 * q1 * q3)],
             [0, 0, 0, 0, 1, 0, dt * (-2 * q1 * (dvz - dvz_b - g) + 2 * q3 * (dvx - dvx_b)),
              dt * (-2 * q0 * (dvz - dvz_b - g) - 4 * q1 * (dvy - dvy_b) + 2 * q2 * (dvx - dvx_b)),
              dt * (2 * q1 * (dvx - dvx_b) + 2 * q3 * (dvz - dvz_b - g)),
              dt * (2 * q0 * (dvx - dvx_b) + 2 * q2 * (dvz - dvz_b - g) - 4 * q3 * (dvy - dvy_b)), 0, 0, 0,
              dt * (-2 * q0 * q3 - 2 * q1 * q2), dt * (2 * q1 ** 2 + 2 * q3 ** 2 - 1),
              dt * (2 * q0 * q1 - 2 * q2 * q3)],
             [0, 0, 0, 0, 0, 1, dt * (2 * q1 * (dvy - dvy_b) - 2 * q2 * (dvx - dvx_b)),
              dt * (2 * q0 * (dvy - dvy_b) - 4 * q1 * (dvz - dvz_b - g) + 2 * q3 * (dvx - dvx_b)),
              dt * (-2 * q0 * (dvx - dvx_b) - 4 * q2 * (dvz - dvz_b - g) + 2 * q3 * (dvy - dvy_b)),
              dt * (2 * q1 * (dvx - dvx_b) + 2 * q2 * (dvy - dvy_b)), 0, 0, 0, dt * (2 * q0 * q2 - 2 * q1 * q3),
              dt * (-2 * q0 * q1 - 2 * q2 * q3), dt * (2 * q1 ** 2 + 2 * q2 ** 2 - 1)],
             [0, 0, 0, 0, 0, 0, 1, -0.5 * dax + 0.5 * dax_b, -0.5 * day + 0.5 * day_b, -0.5 * daz + 0.5 * daz_b,
              0.5 * q1, 0.5 * q2, 0.5 * q3, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0.5 * dax - 0.5 * dax_b, 1, 0.5 * daz - 0.5 * daz_b, -0.5 * day + 0.5 * day_b,
              -0.5 * q0, 0.5 * q3, -0.5 * q2, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0.5 * day - 0.5 * day_b, -0.5 * daz + 0.5 * daz_b, 1, 0.5 * dax - 0.5 * dax_b,
              -0.5 * q3, -0.5 * q0, 0.5 * q1, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0.5 * daz - 0.5 * daz_b, 0.5 * day - 0.5 * day_b, -0.5 * dax + 0.5 * dax_b, 1, 0.5 * q2,
              -0.5 * q1, -0.5 * q0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]
        )

    def create_process_update_matrix(self, dt):
        q0, q1, q2, q3 = self.est_ori
        return np.array(
            [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],
             [0, 0, 0, dt * (-2 * q2 ** 2 - 2 * q3 ** 2 + 1), dt * (-2 * q0 * q3 + 2 * q1 * q2),
              dt * (2 * q0 * q2 + 2 * q1 * q3)],
             [0, 0, 0, dt * (2 * q0 * q3 + 2 * q1 * q2), dt * (-2 * q1 ** 2 - 2 * q3 ** 2 + 1),
              dt * (-2 * q0 * q1 + 2 * q2 * q3)],
             [0, 0, 0, dt * (-2 * q0 * q2 + 2 * q1 * q3), dt * (2 * q0 * q1 + 2 * q2 * q3),
              dt * (-2 * q1 ** 2 - 2 * q2 ** 2 + 1)], [-0.5 * q1, -0.5 * q2, -0.5 * q3, 0, 0, 0],
             [0.5 * q0, -0.5 * q3, 0.5 * q2, 0, 0, 0], [0.5 * q3, 0.5 * q0, -0.5 * q1, 0, 0, 0],
             [-0.5 * q2, 0.5 * q1, 0.5 * q0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]
        )

    def predict(self, measured_angular_vel, measured_acc, dt):
        # A-priori accel and rotation rate estimate
        self.est_acc = measured_acc - self.est_acc_bias
        self.est_rate_rotation = measured_angular_vel - self.est_rate_of_rot_bias

        # Kalman Time Update (Prediction)
        # Compute DCM (Body to/from ENU) Transformations from Quaternion
        local_to_body_trans = quaternion_to_dcm(self.est_ori)
        body_to_local_trans = local_to_body_trans.T

        # Attitude Update
        attitude_delta = np.hstack((1, 0.5 * dt * self.est_rate_rotation))
        self.est_ori = quaternion_mult(self.est_ori, attitude_delta)

        # Avoid quaternion flips sign
        if self.est_ori[0] < 0:
            self.est_ori = -self.est_ori

        # Velocity Update
        gravity_vector = np.array([0.0, 0.0, -self.gravity_const])
        self.est_vel += (body_to_local_trans @ self.est_acc + gravity_vector) * dt

        # Position Update
        self.est_pos += (dt * self.est_vel)

        self.Fx = self.create_state_transition_matrix(dt)

        # Process Noise Covariance (Discrete approximation)
        Fu = self.create_process_update_matrix(dt)

        # Discrete Process Noise
        self.Q = Fu @ self.Rw @ Fu.T
        self.Q = 0.5 * (self.Q + self.Q.T)

        # Covariance Time Update
        self.P = self.Fx @ self.P @ self.Fx.T + self.Q
        self.P = 0.5 * (self.P + self.P.T)

    def correct(self, measured_position, measured_velocity, measured_angular_vel, measured_acc):
        # Kalman Measurement Update
        # Position Error
        pos_err = measured_position - self.est_pos

        # Velocity Error
        vel_err = measured_velocity - self.est_vel

        # Create measurement Y, as Error between Measures and Outputs
        y = np.zeros(6)
        y[0:3] = pos_err
        y[3:6] = vel_err

        # Innovation covariance
        self.S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        self.K = self.P @ self.H.T @ np.linalg.inv(self.S)

        # Covariance update
        I_KH = self._I16 - self.K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + self.K @ self.R @ self.K.T

        # State update, x = K * y
        x = self.K @ y

        # Pull apart x terms to update the Position, velocity, orientation, and sensor biases

        # Position update
        pos_delta = x[0:3]  # Position Deltas in ENU
        self.est_pos += pos_delta

        # Velocity update
        vel_delta = x[3:6]  # Velocity Deltas in ENU
        self.est_vel += vel_delta

        # Attitude correction
        quat_delta = x[6:10]  # Quaternion Delta
        attitude_delta = np.array([quat_delta[0], quat_delta[1], quat_delta[2], quat_delta[3]])
        self.est_ori = quaternion_mult(self.est_ori, attitude_delta)

        # Update biases from states
        acc_bias_delta = x[10:13]  # Accel Bias Deltas
        self.est_acc_bias += acc_bias_delta
        w_bias_delta = x[13:16]  # Rotation Rate Bias Deltas
        self.est_rate_of_rot_bias += w_bias_delta

        # Post-priori accel and rotation rate estimate, biases updated in MeasUpdate()
        self.est_acc = measured_acc - self.est_acc_bias  # only for users convenience
        self.est_rate_rotation = measured_angular_vel - self.est_rate_of_rot_bias  # only for users convenience
