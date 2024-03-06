"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm

def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle


def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    pass


def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix T from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transformation matrix.
    """
    pass


def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the 3 Euler angles from a 4x4 transformation matrix T
                If you like, add an argument to specify the Euler angles used (xyx, zyz, etc.)

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    pass


def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the 6DOF pose vector from a 4x4 transformation matrix T

    @param      T     transformation matrix

    @return     The pose vector from T.
    """
    pass


def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a  representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4x4 homogeneous matrix representing the pose of the desired link

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4x4 homogeneous matrix representing the pose of the desired link
    """
    homo_matrix = np.eye(4)
    for i in range(len(s_lst)):
        w = s_lst[i][:3]
        v = s_lst[i][3:]
        s_matrix = to_s_matrix(w, v)
        exp = expm(s_matrix * joint_angles[i])
        homo_matrix = np.matmul(homo_matrix, exp)
        
    homo_matrix = np.matmul(homo_matrix, m_mat)

    return homo_matrix


def to_s_matrix(w, v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    s_matrix = np.zeros([4, 4])
    s_matrix[0:3, 0:3] = np.array([[0, -w[2], w[1]],
                                   [w[2], 0, -w[0]],
                                   [-w[1], w[0], 0]])
    s_matrix[0:3, 3] = v
    return s_matrix

def IK_geometric(pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose vector as np.array to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose vector as np.array 

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    l1 = 0.10391
    l2 = 0.20573
    l3 = 0.20
    l4 = 0.17415

    theta1 = np.arctan2(-pose[0], pose[1])
    x = np.sqrt(pose[0]**2 + pose[1]**2) 
    y = pose[2] - l1

    X = x - l4 * np.cos(pose[3])
    Y = y - l4 * np.sin(pose[3])

    theta3 = np.arccos(((np.square(X)+np.square(Y))-(np.square(l2)+np.square(l3)))/(2*l2*l3))

    gama1 = np.arctan2(Y, X)
    gama2 = np.arctan2(l3 * np.sin(theta3), l2 + l3 * np.cos(theta3))
    theta2 = np.pi/2 - gama1 - gama2
    
    theta4 = np.pi/2 - pose[3] - theta2 - theta3 
    theta5 = 0 
    return np.array([theta1, theta2 - 0.245, theta3 - 1.325, theta4, theta5], dtype=np.float32) 
