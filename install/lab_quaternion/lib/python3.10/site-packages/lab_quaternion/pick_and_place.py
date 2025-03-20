#!/usr/bin/env python3
import time
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from pyquaternion import Quaternion as PyQuaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import numpy as np


cube_pick_data = np.loadtxt('lab8_pick_data.csv', delimiter=',')
def slerp(data, qStart, qEnd, arm):
    qList = []
    for q in PyQuaternion.intermediates(qStart, qEnd, len(data)-2,
include_endpoints=True):
        qList.append(q.elements)
    for i in range(len(data)):
        pose = Pose()
        pose.position.x = data[i][0]
        pose.position.y = data[i][1]
        pose.position.z = data[i][2]
        pose.orientation.x = qList[i][0]
        pose.orientation.y = qList[i][1]
        pose.orientation.z = qList[i][2]
        pose.orientation.w = qList[i][3]
        arm.inverse_kinematic_movement(pose)
        print(f"Reached control point {i}")


def pick_and_place(pick_data, arm, gripper, qStart, qPick):
    gripper.move_to_position(0.0)
    slerp(pick_data, qStart, qPick, arm)
    time.sleep(0.5)
    print("Got to pick position")
    gripper.move_to_position(0.5)
    time.sleep(0.5)
    pre_put_pose = Pose()
    pre_put_pose.position.x = 0.341
    pre_put_pose.position.y = -0.313
    pre_put_pose.position.z = 0.313
    pre_put_pose.orientation.x = 0.139
    pre_put_pose.orientation.y = 0.990
    pre_put_pose.orientation.z = 0.001
    pre_put_pose.orientation.w = -0.017
    # arm.inverse_kinematic_movement(pre_put_pose)
    time.sleep(0.5)
    
    print("Start to go to put position")
    put_pose = Pose()
    put_pose.position.x = 0.309
    put_pose.position.y = -0.322
    put_pose.position.z = 0.230
    put_pose.orientation.x = 0.771
    put_pose.orientation.y = -0.636
    put_pose.orientation.z = 0.014
    put_pose.orientation.w = -0.004
    arm.inverse_kinematic_movement(put_pose)
    print("Got to put position")
    gripper.move_to_position(0.0)

def main():
    rclpy.init()
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()
    qStartCube = PyQuaternion(array=np.array([-0.665, 0.744, -0.010, -0.054]))
    qPickCube = PyQuaternion(array=np.array([-0.665, 0.746, 0.022,-0.018]))
    pick_and_place(cube_pick_data, arm, gripper, qStartCube, qPickCube)
    rclpy.shutdown()
    gripper.shutdown()
    arm.shutdown()

if __name__ == '__main__':
    main()
