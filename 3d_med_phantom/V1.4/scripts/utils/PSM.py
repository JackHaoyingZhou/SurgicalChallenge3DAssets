from ambf_client import Client
from pathlib import Path

# import os
import sys

sys.path.append(str(Path(__file__).parent))
# dynamic_path = os.path.abspath(__file__+"/../")
# # print(dynamic_path)
# sys.path.append(dynamic_path)
import time
from PyKDL import Vector, Frame, Rotation
from PSM_FK import NewPSMForwardKinematic
from threading import Thread, Lock
import numpy as np


class PSMJointMapping:
    def __init__(self):
        # self.idx_to_name = {0: 'baselink-yawlink',
        #                     1: 'yawlink-pitchendlink',
        #                     2: 'pitchendlink-maininsertionlink',
        #                     3: 'maininsertionlink-toolrolllink',
        #                     4: 'toolrolllink-toolpitchlink',
        #                     5: 'toolpitchlink-toolyawlink',
        #                     6: 'toolyawlink-toolgripperleftlink',
        #                     7: 'toolyawlink-toolgripperrightlink'}
        self.idx_to_name = {
            0: "baselink-yawlink",
            1: "yawlink-pitchendlink",
            2: "pitchendlink-maininsertionlink",
            3: "maininsertionlink-toolrolllink",
            4: "toolrolllink-toolpitchlink",
            5: "toolpitchlink-toolyawlink",
        }

        # self.name_to_idx = {'baselink-yawlink': 0,
        #                     'yawlink-pitchendlink': 1,
        #                     'pitchendlink-maininsertionlink': 2,
        #                     'maininsertionlink-toolrolllink': 3,
        #                     'toolrolllink-toolpitchlink': 4,
        #                     'toolpitchlink-toolyawlink': 5,
        #                     'toolyawlink-toolgripperleftlink': 6,
        #                     'toolyawlink-toolgripperrightlink': 7}
        self.name_to_idx = {
            "baselink-yawlink": 0,
            "yawlink-pitchendlink": 1,
            "pitchendlink-maininsertionlink": 2,
            "maininsertionlink-toolrolllink": 3,
            "toolrolllink-toolpitchlink": 4,
            "toolpitchlink-toolyawlink": 5,
        }


pjm = PSMJointMapping()


class NewPSM:
    def __init__(self, client: Client, arm_name: str):
        self.client = client
        time.sleep(0.5)
        self.base = self.client.get_obj_handle(arm_name + "/baselink")
        time.sleep(0.5)

        ### base transformation info
        self.T_t_b_home = Frame(Rotation.RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0))
        self._kd = NewPSMForwardKinematic()

        ## init
        self._T_b_w = None
        self._T_w_b = None
        self._base_pose_updated = False
        self._num_joints = 6
        self._ik_solution = np.zeros([self._num_joints])
        self._force_exit_thread = False
        self._thread_lock = Lock()
        # self.set_jaw_angle(0.5)
        time.sleep(0.5)

    def set_home_pose(self, pose):
        self.T_t_b_home = pose

    def is_present(self):
        if self.base is None:
            return False
        else:
            return True

    def get_lower_limits(self):
        return self._kd.lower_limits

    def get_upper_limits(self):
        return self._kd.upper_limits

    def get_T_w_b(self):
        self.__update_base_pose()
        return self._T_w_b

    def __update_base_pose(self):
        if not self._base_pose_updated:
            self._T_b_w = self.base.get_pose()
            self._T_w_b = self._T_b_w.Inverse()
            self._base_pose_updated = True

    def servo_cp(self, T_t_b):
        pass

    def servo_jp(self, jp):
        self.base.set_joint_pos(0, jp[0])
        self.base.set_joint_pos(1, jp[1])
        self.base.set_joint_pos(2, jp[2])
        self.base.set_joint_pos(3, jp[3])
        self.base.set_joint_pos(4, jp[4])
        self.base.set_joint_pos(5, jp[5])

    def servo_jv(self, jv):
        print("Setting Joint Vel: ", jv)
        self.base.set_joint_vel(0, jv[0])
        self.base.set_joint_vel(1, jv[1])
        self.base.set_joint_vel(2, jv[2])
        self.base.set_joint_vel(3, jv[3])
        self.base.set_joint_vel(4, jv[4])
        self.base.set_joint_vel(5, jv[5])

    def set_jaw_angle(self, jaw_angle):
        self.base.set_joint_pos(6, -jaw_angle)
        self.base.set_joint_pos(7, -jaw_angle)

    def measured_jp(self):
        j0 = self.base.get_joint_pos(0)
        j1 = self.base.get_joint_pos(1)
        j2 = self.base.get_joint_pos(2)
        j3 = self.base.get_joint_pos(3)
        j4 = self.base.get_joint_pos(4)
        j5 = self.base.get_joint_pos(5)
        q = [j0, j1, j2, j3, j4, j5]
        return q

    def measured_jv(self):
        j0 = self.base.get_joint_vel(0)
        j1 = self.base.get_joint_vel(1)
        j2 = self.base.get_joint_vel(2)
        j3 = self.base.get_joint_vel(3)
        j4 = self.base.get_joint_vel(4)
        j5 = self.base.get_joint_vel(5)
        return [j0, j1, j2, j3, j4, j5]

    def get_joint_names(self):
        return self.base.get_joint_names()
