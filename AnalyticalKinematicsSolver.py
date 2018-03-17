import numpy as np
from collections import namedtuple
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class AnalyticalKinematicsSolver:

    """
    Analytical inverse kinematics solver for 6-axis industrial serial manipulators
    """

    def __init__(self, robot_type, custom_paramset=None):
        """
        Create robot instance with parameters
        """
        ParamSet = namedtuple('ParamSet', ('a1', 'a2', 'b', 'c1', 'c2', 'c3', 'c4'))

        if robot_type == "kuka_youbot_arm":
            self.param_set = ParamSet(0.033, 0.0,  0.0,  0.147, 0.155, 0.135, 0.2175)
        elif robot_type == "katana_450_6M180":
            self.param_set = ParamSet(0.0, 0.0,  0.0,  0.2015, 0.190, 0.139, 0.1883)
        elif robot_type == "adept_viper_s650":
            self.param_set = ParamSet(0.075, -0.09,  0.0,  0.335, 0.270, 0.295, 0.08)
        elif robot_type == "custom":
            self.param_set = custum_paramset
        else:
            raise ValueError("Given robot type %s is not supported" % robot_type)

    def getWristCenterOrientation(self, theta):
        """
        Return rotation matrix of wrist center R0c
        """
        r11 = np.cos(theta[0])*np.cos(theta[1])*np.cos(theta[2]) - np.cos(theta[0])*np.sin(theta[1])*np.sin(theta[2])
        r12 = -np.sin(theta[0])
        r13 = np.cos(theta[0])*np.cos(theta[1])*np.sin(theta[2]) + np.cos(theta[0])*np.sin(theta[1])*np.cos(theta[2])
        r21 = np.sin(theta[0])*np.cos(theta[1])*np.cos(theta[2]) - np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2])
        r22 = np.cos(theta[0])
        r23 = np.sin(theta[0])*np.cos(theta[1])*np.sin(theta[2]) + np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2])
        r31 = -np.sin(theta[1])*np.cos(theta[2]) - np.cos(theta[1])*np.sin(theta[2])
        r32 = 0.0
        r33 = -np.sin(theta[1])*np.sin(theta[2]) + np.cos(theta[1])*np.cos(theta[2])
        R = np.matrix([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])

        return R


    def getEndEffectorRelativeOrientation(self, theta):
        """
        Return rotation matrix of end effecter w.r.t. wrist center Rce
        """
        r11 = np.cos(theta[3]) * np.cos(theta[4]) * np.cos(theta[5]) - np.sin(theta[3]) * np.sin(theta[5])
        r12 = - np.cos(theta[3]) * np.cos(theta[4]) * np.sin(theta[5]) - np.sin(theta[3]) * np.cos(theta[5])
        r13 = np.cos(theta[3]) * np.sin(theta[4])
        r21 = np.sin(theta[3]) * np.cos(theta[4]) * np.cos(theta[5]) + np.cos(theta[3]) * np.sin(theta[5])
        r22 = -np.sin(theta[3]) * np.cos(theta[4]) * np.sin(theta[5]) + np.cos(theta[3]) * np.cos(theta[5])
        r23 = np.sin(theta[3]) * np.sin(theta[4])
        r31 = -np.sin(theta[4]) * np.cos(theta[5])
        r32 = np.sin(theta[4]) * np.sin(theta[5])
        r33 = np.cos(theta[4])
        R = np.matrix([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])

        return R

    def getRollPitchYawOrientationMatrix(self, roll, pitch, yaw):
        """
        Get rotation matrix
        """
        r11 = np.cos(yaw) * np.cos(pitch)
        r12 = np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll)
        r13 = np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)
        r21 = np.sin(yaw) * np.cos(pitch)
        r22 = np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll)
        r23 = np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)
        r31 = - np.sin(pitch)
        r32 = np.cos(pitch) * np.sin(roll)
        r33 = np.cos(pitch) * np.cos(roll)
        R = np.matrix([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])

        return R

    def getRollPitchYawAngle(self, R):
        """
        Get roll, pitch and yaw angle from rotation matrix
        """

        yaw = np.arctan2(R[1, 0], R[0, 0])
        #yaw = np.arctan2(R[1, 0], R[0, 0]) + np.pi
        #yaw = np.arctan2(R[1, 0], R[0, 0]) - np.pi

        pitch = np.arctan2(-R[2, 0],  R[0,  0] * np.cos(yaw) + R[1,  0] * np.sin(yaw))

        roll = np.arctan2(R[0, 2] * np.sin(yaw) - R[1, 2] * np.cos(yaw), -R[0, 1] * np.sin(yaw) + R[1, 1] * np.cos(yaw) )

        return roll, pitch, yaw

    # TODO: Implement quartanion

    def solveForwardKinematics(self, theta):
        """
        Return end-effector position vector (ux, uy, uz) and orientation matrix R0e
        """

        R0c = self.getWristCenterOrientation(theta)
        Rce = self.getEndEffectorRelativeOrientation(theta)
        R0e = R0c * Rce

        phi3 = np.arctan2(self.param_set.a2, self.param_set.c3)
        k = np.sqrt(self.param_set.a2**2 + self.param_set.c3**2)

        cx1 = self.param_set.c2 * np.sin(theta[1]) + k * np.sin(theta[1] + theta[2] + phi3) + self.param_set.a1
        cy1 = self.param_set.b
        cz1 = self.param_set.c2 * np.cos(theta[1]) + k * np.cos(theta[1] + theta[2] + phi3)
        c1 = np.matrix([[cx1],  [cy1],  [cz1]])

        cx0 = c1[0, 0] * np.cos(theta[0]) - c1[1,  0] * np.sin(theta[0])
        cy0 = c1[0, 0] * np.sin(theta[0]) + c1[1,  0] * np.cos(theta[0])
        cz0 = c1[2, 0] + self.param_set.c1
        c0 = np.matrix([[cx0],  [cy0],  [cz0]])

        u = c0 + self.param_set.c4 * R0e * np.matrix([[0], [0], [1]])

        return u, R0e


    def solveInverseKinematics(self, u, R0e):
        """
        Return inverse kinematics solution given u and R0e
        """
        c = u - self.param_set.c4 * R0e * np.matrix([[0], [0], [1]])
        n_x1 = np.sqrt(c[0, 0]**2 + c[1, 0]**2 - self.param_set.b**2) - self.param_set.a1
        s1_sq = n_x1**2 + (c[2, 0] - self.param_set.c1)**2
        s2_sq = (n_x1 + 2.0 * self.param_set.a1)**2 + (c[2, 0] - self.param_set.c1)**2
        k_sq = self.param_set.a2**2 + self.param_set.c3**2

        theta1_1 = np.arctan2(c[1, 0], c[0, 0]) - np.arctan2(self.param_set.b, n_x1 + self.param_set.a1)
        theta1_2 = np.arctan2(c[1, 0], c[0, 0]) + np.arctan2(self.param_set.b, n_x1 + self.param_set.a1) - np.pi

        theta2_1 = -np.arccos( (s1_sq + self.param_set.c2**2 - k_sq) / (2.0 * np.sqrt(s1_sq) * self.param_set.c2 ) ) + np.arctan2(n_x1, c[2, 0] - self.param_set.c1)
        theta2_2 = np.arccos( (s1_sq + self.param_set.c2**2 - k_sq) / (2.0 * np.sqrt(s1_sq) * self.param_set.c2 ) ) + np.arctan2(n_x1, c[2, 0] - self.param_set.c1)

        theta2_3 = -np.arccos( (s2_sq + self.param_set.c2**2 - k_sq) / (2.0 * np.sqrt(s2_sq) * self.param_set.c2 ) ) - np.arctan2(n_x1+2.0*self.param_set.a1, c[2, 0] - self.param_set.c1)
        theta2_4 = np.arccos( (s2_sq + self.param_set.c2**2 - k_sq) / (2.0 * np.sqrt(s2_sq) * self.param_set.c2 ) ) - np.arctan2(n_x1+2.0*self.param_set.a1, c[2, 0] - self.param_set.c1)

        theta3_1 = np.arccos( (s1_sq - self.param_set.c2**2 - k_sq) / (2.0 * self.param_set.c2 * np.sqrt(k_sq) ) ) - np.arctan2(self.param_set.a2, self.param_set.c3)
        theta3_2 = -np.arccos( (s1_sq - self.param_set.c2**2 - k_sq) / (2.0 * self.param_set.c2 * np.sqrt(k_sq) ) ) - np.arctan2(self.param_set.a2, self.param_set.c3)

        theta3_3 = np.arccos( (s2_sq - self.param_set.c2**2 - k_sq) / (2.0 * self.param_set.c2 * np.sqrt(k_sq) ) ) - np.arctan2(self.param_set.a2, self.param_set.c3)
        theta3_4 = -np.arccos( (s2_sq - self.param_set.c2**2 - k_sq) / (2.0 * self.param_set.c2 * np.sqrt(k_sq) ) ) - np.arctan2(self.param_set.a2, self.param_set.c3)


        s1_1 = np.sin(theta1_1)
        s1_2 = np.sin(theta1_2)
        s1_3 = np.sin(theta1_1)
        s1_4 = np.sin(theta1_2)

        c1_1 = np.cos(theta1_1)
        c1_2 = np.cos(theta1_2)
        c1_3 = np.cos(theta1_1)
        c1_4 = np.cos(theta1_2)

        s23_1 = np.sin(theta2_1 + theta3_1)
        s23_2 = np.sin(theta2_2 + theta3_2)
        s23_3 = np.sin(theta2_3 + theta3_3)
        s23_4 = np.sin(theta2_4 + theta3_4)

        c23_1 = np.cos(theta2_1 + theta3_1)
        c23_2 = np.cos(theta2_2 + theta3_2)
        c23_3 = np.cos(theta2_3 + theta3_3)
        c23_4 = np.cos(theta2_4 + theta3_4)

        m1 = R0e[0, 2] * s23_1 * c1_1 + R0e[1,  2] * s23_1 * s1_1 + R0e[2,  2] * c23_1
        m2 = R0e[0, 2] * s23_2 * c1_2 + R0e[1,  2] * s23_2 * s1_2 + R0e[2,  2] * c23_2
        m3 = R0e[0, 2] * s23_3 * c1_3 + R0e[1,  2] * s23_3 * s1_3 + R0e[2,  2] * c23_3
        m4 = R0e[0, 2] * s23_4 * c1_4 + R0e[1,  2] * s23_4 * s1_4 + R0e[2,  2] * c23_4

        theta4_1 = np.arctan2(R0e[1, 2] * c1_1 - R0e[0, 2] * s1_1, R0e[0, 2] * c23_1 * c1_1 + R0e[1, 2] * c23_1 * s1_1 - R0e[2, 2] * s23_1)
        theta4_2 = np.arctan2(R0e[1, 2] * c1_2 - R0e[0, 2] * s1_2, R0e[0, 2] * c23_2 * c1_2 + R0e[1, 2] * c23_2 * s1_2 - R0e[2, 2] * s23_2)
        theta4_3 = np.arctan2(R0e[1, 2] * c1_3 - R0e[0, 2] * s1_3, R0e[0, 2] * c23_3 * c1_3 + R0e[1, 2] * c23_3 * s1_3 - R0e[2, 2] * s23_3)
        theta4_4 = np.arctan2(R0e[1, 2] * c1_4 - R0e[0, 2] * s1_4, R0e[0, 2] * c23_4 * c1_4 + R0e[1, 2] * c23_4 * s1_4 - R0e[2, 2] * s23_4)

        theta4_5 = theta4_1 + np.pi
        theta4_6 = theta4_2 + np.pi
        theta4_7 = theta4_3 + np.pi
        theta4_8 = theta4_4 + np.pi

        theta5_1 = np.arctan2(np.sqrt(1-m1**2), m1)
        theta5_2 = np.arctan2(np.sqrt(1-m2**2), m2)
        theta5_3 = np.arctan2(np.sqrt(1-m3**2), m3)
        theta5_4 = np.arctan2(np.sqrt(1-m4**2), m4)

        theta5_5 = -theta5_1
        theta5_6 = -theta5_2
        theta5_7 = -theta5_3
        theta5_8 = -theta5_4

        theta6_1 = np.arctan2(R0e[0, 1] * s23_1 * c1_1 + R0e[1, 1] * s23_1 * s1_1 + R0e[2, 1] * c23_1, -R0e[0, 0] * s23_1 * c1_1 - R0e[1, 0] * s23_1 * s1_1 - R0e[2, 0] * c23_1)
        theta6_2 = np.arctan2(R0e[0, 1] * s23_2 * c1_2 + R0e[1, 1] * s23_2 * s1_2 + R0e[2, 1] * c23_2, -R0e[0, 0] * s23_2 * c1_2 - R0e[1, 0] * s23_2 * s1_2 - R0e[2, 0] * c23_2)
        theta6_3 = np.arctan2(R0e[0, 1] * s23_3 * c1_3 + R0e[1, 1] * s23_3 * s1_3 + R0e[2, 1] * c23_3, -R0e[0, 0] * s23_3 * c1_3 - R0e[1, 0] * s23_3 * s1_3 - R0e[2, 0] * c23_3)
        theta6_4 = np.arctan2(R0e[0, 1] * s23_4 * c1_4 + R0e[1, 1] * s23_4 * s1_4 + R0e[2, 1] * c23_4, -R0e[0, 0] * s23_4 * c1_4 - R0e[1, 0] * s23_4 * s1_4 - R0e[2, 0] * c23_4)

        theta6_5 = theta6_1 - np.pi
        theta6_6 = theta6_2 - np.pi
        theta6_7 = theta6_3 - np.pi
        theta6_8 = theta6_4 - np.pi

        theta_set1 = [theta1_1, theta2_1, theta3_1, theta4_1, theta5_1, theta6_1] # lefty  - above  - nonflip
        theta_set2 = [theta1_1, theta2_2, theta3_2, theta4_2, theta5_2, theta6_2] # lefty  - bellow - nonflip
        theta_set3 = [theta1_2, theta2_3, theta3_3, theta4_3, theta5_3, theta6_3] # righty - bellow - nonflip
        theta_set4 = [theta1_2, theta2_4, theta3_4, theta4_4, theta5_4, theta6_4] # righty - above  - flip
        theta_set5 = [theta1_1, theta2_1, theta3_1, theta4_5, theta5_5, theta6_5] # lefty  - above  - flip
        theta_set6 = [theta1_1, theta2_2, theta3_2, theta4_6, theta5_6, theta6_6] # lefty  - bellow - flip
        theta_set7 = [theta1_2, theta2_3, theta3_3, theta4_7, theta5_7, theta6_7] # righty - bellow - flip
        theta_set8 = [theta1_2, theta2_4, theta3_4, theta4_8, theta5_8, theta6_8] # righty - above  - nonflip

        return theta_set1, theta_set2, theta_set3, theta_set4, theta_set5, theta_set6, theta_set7, theta_set8

if __name__ == '__main__':

    # Instanciation
    kinematics_solver = AnalyticalKinematicsSolver("adept_viper_s650")

    # End-effector FK/IK example code
    theta = [0.0*np.pi/180.0, 30.0*np.pi/180.0, 90.0*np.pi/180.0,  0.0,  10.0*np.pi/180.0, 0.0] # (lefty, above, nonflip)

    # Calcuate forward kinematics solution
    pos_fk, R_fk = kinematics_solver.solveForwardKinematics(theta)

    # Get roll, pitch, yaw angle
    roll, pitch, yaw = kinematics_solver.getRollPitchYawAngle(R_fk)

    # Get roll, pitch, yaw orientation matrix
    R_rpy = kinematics_solver.getRollPitchYawOrientationMatrix(roll,  pitch,  yaw)

    # Verify the forward kinematics solution by inverse kinematics
    theta_ik1, theta_ik2, theta_ik3, theta_ik4, theta_ik5, theta_ik6, theta_ik7, theta_ik8 = kinematics_solver.solveInverseKinematics(pos_fk, R_rpy)

    print(pos_fk)
    print(theta)
    print(theta_ik1)


