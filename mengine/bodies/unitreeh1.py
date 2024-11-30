import os
import pybullet as p
from .robot import Robot


class UnitreeH1(Robot):
    def __init__(self, env, position=[0, 0, 0], orientation=[0, 0, 0, 1], controllable_joints=None, fixed_base=True):
        end_effector = 57 # dummy
        gripper_joints = [55, 56] # dummy
        body = p.loadURDF(os.path.join(env.directory, 'h1_2_description', 'h1_2.urdf'), useFixedBase=fixed_base, basePosition=position, baseOrientation=orientation, physicsClientId=env.id)
        joints = []
        for i in range(p.getNumJoints(body, physicsClientId=env.id)):
            joint_type = p.getJointInfo(body, i, physicsClientId=env.id)[2]
            if joint_type != p.JOINT_FIXED:
                joints.append(i)
        controllable_joints = joints if controllable_joints is None else controllable_joints
        super().__init__(body, env, controllable_joints, end_effector, gripper_joints)
