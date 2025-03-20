#!/usr/bin/env python3
import time
import rclpy
import numpy as np
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion as PyQuaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper

# Load pick trajectory data
PICK_TRAJECTORY_DATA = np.loadtxt('lab8_pick_data.csv', delimiter=',')

def perform_slerp_motion(trajectory_data, start_quat, end_quat, arm):
    """
    Performs spherical linear interpolation (SLERP) to generate intermediate orientations
    and moves the robotic arm along a given trajectory.

    :param trajectory_data: List of position waypoints for movement
    :param start_quat: Starting quaternion orientation
    :param end_quat: Target quaternion orientation
    :param arm: Instance of the robotic arm controller
    """
    interpolated_quaternions = [
        q.elements for q in PyQuaternion.intermediates(start_quat, end_quat, len(trajectory_data) - 2, include_endpoints=True)
    ]

    for i, (position, quaternion) in enumerate(zip(trajectory_data, interpolated_quaternions)):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        arm.inverse_kinematic_movement(pose)
        print(f"Reached waypoint {i}")

def execute_pick_and_place(trajectory_data, arm, gripper, start_quat, pick_quat):
    """
    Executes a pick-and-place sequence using the robotic arm and gripper.

    :param trajectory_data: List of positions for the picking path
    :param arm: Instance of the robotic arm controller
    :param gripper: Instance of the robotic gripper controller
    :param start_quat: Starting quaternion for the trajectory
    :param pick_quat: Quaternion for the pick position
    """
    # Open gripper
    gripper.move_to_position(0.0)

    # Move along the pick trajectory
    perform_slerp_motion(trajectory_data, start_quat, pick_quat, arm)
    time.sleep(0.5)
    print("Reached pick position")

    # Close gripper to grasp the object
    gripper.move_to_position(0.5)
    time.sleep(0.5)

    print("Moving to place position")
    move_to_pose(arm, [0.309, -0.322, 0.230], [0.771, -0.636, 0.014, -0.004])
    print("Reached place position")

    # Release the object
    gripper.move_to_position(0.0)

def move_to_pose(arm, position, orientation):
    """
    Moves the robotic arm to a specified pose.

    :param arm: Instance of the robotic arm controller
    :param position: List of [x, y, z] coordinates
    :param orientation: List of [qx, qy, qz, qw] quaternion orientation
    """
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    arm.inverse_kinematic_movement(pose)

def main():
    """
    Initializes the robotic arm and gripper, then executes the pick-and-place task.
    """
    rclpy.init()
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()

    # Define quaternions for different motion stages
    initial_quat = PyQuaternion(array=np.array([-0.665, 0.744, -0.010, -0.054]))
    pick_quat = PyQuaternion(array=np.array([-0.665, 0.746, 0.022, -0.018]))

    execute_pick_and_place(PICK_TRAJECTORY_DATA, arm, gripper, initial_quat, pick_quat)

    # Shutdown procedures
    rclpy.shutdown()
    gripper.shutdown()
    arm.shutdown()

if __name__ == '__main__':
    main()

