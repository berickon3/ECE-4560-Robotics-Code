import numpy as np
from scipy.spatial.transform import Rotation as Rot

def PD_control(d, q_des, dq_des):
    """
    PD Control
    :param d: Mujoco Data
    :param q_des: Desired Joint Angles
    :param dq_des: Desired Joint Velocities
    :return: Torques
    """
    # Proportional Gain
    Kp = np.array([200, 200, 200, 100, 100, 100])
    
    # Derivative Gain
    Kd = np.array([10, 10, 10, 10, 10, 10])
    
    # Number of Joints
    nj = d.qpos.shape[0]
    # Torques
    torques = np.zeros(nj)
    # Loop through each joint
    for i in range(nj):
        # Compute the Torque
        torques[i] = (Kp[i]*(q_des[i] - d.qpos[i])) + (Kd[i] * (dq_des[i] - d.qvel[i]))
    return torques

def rodrigues_formula(w, theta):
    """Compute the matrix exponential for a pure rotation."""
    w_skew = skew_symmetric(w)
    I = np.eye(3)
    return I + np.sin(theta) * w_skew + (1 - np.cos(theta)) * np.dot(w_skew, w_skew)

def twist_exponential_hand(w, v, theta):
    """
    Compute the matrix exponential of a twist matrix using its rotational and translational components.
    
    Parameters:
    - w: The angular velocity vector (3x1 numpy array).
    - v: The linear velocity vector (3x1 numpy array).
    - theta: The joint angle (or displacement for prismatic joint).
    
    Returns:
    - A 4x4 homogeneous transformation matrix representing the twist exponential.
    """
    # Compute the rotation part
    R = rodrigues_formula(w, theta)

    # Compute the translation part
    w_skew = skew_symmetric(w)
    I = np.eye(3)
    P = (I - R).dot(np.dot(w_skew, v)) + np.outer(w, w).dot(v) * theta

    # Construct the 4x4 transformation matrix
    twist_exp = np.eye(4)
    twist_exp[:3, :3] = R
    twist_exp[:3, 3] = P

    return twist_exp

def skew_symmetric(w):
    w = w.flatten()
    return np.array([
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0]
    ])

def un_skew_symmetric(w_skew):
    return np.array([w_skew[2, 1], w_skew[0, 2], w_skew[1, 0]])

def unhat(twist_hat):
    """
    Convert a twist matrix to a twist vector.
    :param twist_hat: Twist matrix
    :return: Twist vector
    """
    twist_unhat = np.zeros(6)
    twist_unhat[0] = twist_hat[0, 3]
    twist_unhat[1] = twist_hat[1, 3]
    twist_unhat[2] = twist_hat[2, 3]
    twist_unhat[3] = twist_hat[2, 1]
    twist_unhat[4] = twist_hat[0, 2]
    twist_unhat[5] = twist_hat[1, 0]
    
    return twist_unhat.reshape((6, 1))

def twist_matrix(v, w):
    w_skew = skew_symmetric(w)
    return np.block([
        [w_skew, np.reshape(v, (3, 1))],
        [0, 0, 0, 0]
    ])
       
def rot_x(theta):
    """
    Rotation Matrix about X Axis
    :param theta: Angle in Radians
    :return: Rotation Matrix
    """
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])
                    
def rot_y(theta):
    """
    Rotation Matrix about Y Axis
    :param theta: Angle in Radians
    :return: Rotation Matrix
    """
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def rot_z(theta):
    """
    Rotation Matrix about Z Axis
    :param theta: Angle in Radians
    :return: Rotation Matrix
    """
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])
    
def rot_to_eul(R):
    """
    Convert a rotation matrix to Euler angles.
    :param R: Rotation matrix
    :return: Euler angles (phi, theta, psi)
    """
    # sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    # singular = sy < 1e-6

    # if not singular:
    #     x = np.arctan2(R[2, 1], R[2, 2])
    #     y = np.arctan2(-R[2, 0], sy)
    #     z = np.arctan2(R[1, 0], R[0, 0])
    # else:
    #     x = np.arctan2(-R[1, 2], R[1, 1])
    #     y = np.arctan2(-R[2, 0], sy)
    #     z = 0
    
    r = Rot.from_matrix(R)
    euler_angles = r.as_euler('xyz', degrees=False)

    return np.array([euler_angles[0], euler_angles[1], euler_angles[2]])

def eul_to_rot(eul):
    """
    Convert Euler angles to a rotation matrix.
    :param eul: Euler angles (phi, theta, psi)
    :return: Rotation matrix
    """
    return Rot.from_euler('xyz', eul, degrees=False).as_matrix()

def adjoint_matrix(twist_matrix):
    """
    Compute the adjoint representation of a twist matrix.
    :param twist_matrix: Twist matrix
    :return: Adjoint matrix
    """
    R = twist_matrix[0:3, 0:3]
    p = twist_matrix[0:3, 3].reshape((3, 1))
    p_skew = skew_symmetric(p)
    
    adjoint = np.block([[R, np.zeros((3, 3))],
                        [p_skew @ R, R]])
    
    return adjoint