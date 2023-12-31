import numpy as np

def euler_to_quaternion(s):
    """
    Euler angles (3-2-1) to quaternion
    """
    phi = s[0]
    theta = s[1]
    psi = s[2]
    q = np.zeros(4)
    q[0] = np.sin(phi / 2) * np.sin(theta / 2) * np.sin(psi / 2) + np.cos(phi / 2) * np.cos(theta / 2) * np.cos(psi / 2)
    q[1] = np.sin(phi / 2) * np.cos(theta / 2) * np.cos(psi / 2) - np.cos(phi / 2) * np.sin(theta / 2) * np.sin(psi / 2)
    q[2] = np.cos(phi / 2) * np.sin(theta / 2) * np.cos(psi / 2) + np.sin(phi / 2) * np.cos(theta / 2) * np.sin(psi / 2)
    q[3] = np.cos(phi / 2) * np.cos(theta / 2) * np.sin(psi / 2) - np.sin(phi / 2) * np.sin(theta / 2) * np.cos(psi / 2)
    return q

def quaternion_to_dcm(q):
    """
    Quaternion to DCM, S&L 3rd (1.8-18)
    :param q: attitude in quaternions
    :return:
    """
    T = np.zeros((3, 3))
    T[0, 0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]
    T[1, 1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]
    T[2, 2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
    T[0, 1] = 2 * (q[1] * q[2] + q[0] * q[3])
    T[0, 2] = 2 * (q[1] * q[3] - q[0] * q[2])
    T[1, 0] = 2 * (q[1] * q[2] - q[0] * q[3])
    T[1, 2] = 2 * (q[2] * q[3] + q[0] * q[1])
    T[2, 0] = 2 * (q[1] * q[3] + q[0] * q[2])
    T[2, 1] = 2 * (q[2] * q[3] - q[0] * q[1])
    return T

def normalize_quaternion(q):
    """
    Rotation Transforms, Quaternions
    :param q: attitude in quaternions
    :return:
    """
    qNorm = np.linalg.norm(q)
    if qNorm != 0:
        q = q / qNorm
    return q


def quaternion_to_euler(q):
    """
    Quaternion to Euler angles (3-2-1)
    :param q:
    :return:
    """
    q = normalize_quaternion(q)
    T = quaternion_to_dcm(q)
    phi = np.arctan2(T[1, 2], T[2, 2])
    theta = np.arcsin(-T[0, 2])
    psi = np.arctan2(T[0, 1], T[0, 0])
    # Make psi be 0 to 2*pi, rather than -pi to pi
    psi = np.mod(psi, 2 * np.pi)
    s = np.array([phi, theta, psi])
    return s


def quaternion_mult(lhs, rhs):
    """
    Quaternion multiplication
    :param lhs: 
    :param rhs: 
    :return: 
    """
    p0, p1, p2, p3 = lhs
    P = np.array([
        [p0, -p1, -p2, -p3],
        [p1, p0, -p3, p2],
        [p2, p3, p0, -p1],
        [p3, -p2, p1, p0]])
    return normalize_quaternion(P @ rhs)
