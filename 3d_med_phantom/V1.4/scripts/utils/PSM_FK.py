import os
import sys
dynamic_path = os.path.abspath(__file__+"/../../")
# print(dynamic_path)
sys.path.append(dynamic_path)
import numpy as np
from typing import Union
from utility import twist2ht, skew


class NewPSMForwardKinematic:
    def __init__(self):
        self.num_link = 6 #8 if adding two gripper

        ### the following values are readings from Blender
        self.link_RCM_toolmain = 0.4318
        self.link_RCM_toolroll = 0.38
        self.link_RCM_toolpitch = -0.050821
        self.link_RCM_toolyaw = -0.059813
        self.screw_axis =  self.__get_screw_axis()
        self.M = self.__get_M()
        self.lower_limits = [np.deg2rad(-90), np.deg2rad(-60), 0.00, np.deg2rad(-175), np.deg2rad(-90), np.deg2rad(-85), np.deg2rad(-90), np.deg2rad(-90)]
        self.upper_limits = [np.deg2rad(90), np.deg2rad(60), 0.24, np.deg2rad(175), np.deg2rad(90), np.deg2rad(85), np.deg2rad(90), np.deg2rad(90)]

    def __get_screw_axis(self)->np.ndarray:
        '''
        Obtain the screw axes for the robot model
        :return: nx6 matrix for screw axes of all n joints
        '''
        self.w1 = np.array([-1, 0, 0])
        self.w2 = np.array([0, 1, 0])
        self.w3 = np.array([0, 0, 0])
        self.w4 = np.array([0, 0, -1])
        self.w5 = np.array([0, 1, 0])
        self.w6 = np.array([-1, 0 ,0])
        self.w7 = np.array([1, 0, 0])
        self.w8 = np.array([1, 0, 0])
        self.p1 = np.array([0, 0, 0])
        self.p2 = np.array([0, 0, 0])
        self.p3 = np.array([0, 0, self.link_RCM_toolmain])
        self.p4 = np.array([0, 0, self.link_RCM_toolroll])
        self.p5 = np.array([0, 0, self.link_RCM_toolpitch])
        self.p6 = np.array([0, 0, self.link_RCM_toolyaw])
        self.p7 = self.p6
        self.p8 = self.p6

        for i_v in range(self.num_link):
            if i_v + 1 == 3:
                self.v3 = np.array([0, 0, -1])
            else:
                exec(f'self.v{i_v+1} = -skew(self.w{i_v+1}) @ self.p{i_v+1}')

        screw_axes = np.zeros((self.num_link, 6))
        for i_s in range(self.num_link):
            exec(f'screw_axes[{i_s}, :] = np.hstack((self.w{i_s+1}, self.v{i_s+1}))')

        return screw_axes

    def __get_M(self)->np.ndarray:
        '''
        Find the transformation matrix M between the robot base and the end-effector
        :return: a 4x4 homogeneous transformation matrix
        '''
        M = np.array([[0, 0, 1, 0],
                      [1, 0, 0, 0],
                      [0, 1, 0, self.link_RCM_toolyaw],
                      [0, 0, 0, 1]])
        return M

    def compute_FK(self, joint_val:Union[np.array, list])->np.ndarray:
        '''
        Compute the forward kinematics for the robot
        :param joint_val: joint values (angle in rad, distance in meter)
        :return: a 4x4 homogeneous transformation matrix
        '''
        assert len(joint_val) == self.num_link, 'The FK input should have the same length as the number of robot DOF'
        T = np.eye(4)
        for i_axis in range(self.num_link):
            S = self.screw_axis[i_axis, :]
            q = joint_val[i_axis]
            T_i = twist2ht(S, q)
            T = np.dot(T, T_i)
        T = np.dot(T, self.M)
        return T


if __name__ == "__main__":
    lnd = NewPSMForwardKinematic()
    T_0 = lnd.compute_FK([0, 0, 0, 0, 0 ,0])
    print('init pose: \n')
    print(T_0)