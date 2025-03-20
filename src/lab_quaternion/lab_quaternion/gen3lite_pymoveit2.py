#!/usr/bin/env python3

# based on ex_pose_goal.py and ex_gripper.py from pymoveit2 package
# https://github.com/AndrejOrsula/pymoveit2/blob/master/examples/ex_pose_goal.py
# https://github.com/AndrejOrsula/pymoveit2/blob/master/examples/ex_gripper.py

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.exceptions
import rclpy.executors
from rclpy.node import Node
from pymoveit2 import GripperInterface, MoveIt2, MoveIt2State
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
import time


class Gen3LiteGripper():
    def __init__(self):
        self.node = Node("gen3_lite_gripper")
        self.callback_group = ReentrantCallbackGroup()
        self.min_close = 0.0
        self.max_open = 0.85
        self.gripper = GripperInterface(
            node=self.node,
            gripper_joint_names=['right_finger_bottom_joint',],
            open_gripper_joint_positions=[self.max_open,],
            closed_gripper_joint_positions=[self.min_close,],
            gripper_group_name='gripper',
            callback_group=self.callback_group,
            gripper_command_action_name='gripper_action_controller/gripper_cmd',
        )
        # spin node in background and wait for ini
        self.executor = rclpy.executors.MultiThreadedExecutor(2)
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        self.executor_thread.start()
        self.node.create_rate(1.0).sleep()

    def open(self):
        """open gripper"""
        self.node.get_logger().info('open gripper')
        self.gripper.open()
        self.gripper.wait_until_executed()
        time.sleep(1)
    
    def close(self):
        """close gripper"""
        self.node.get_logger().info('close gripper')
        self.gripper.close()
        self.gripper.wait_until_executed()
        time.sleep(1)
    
    def move_to_position(self, position):
        """
        move gripper to desired position
        position: float between 0.0 and 0.85
        returns: None
        """
        self.node.get_logger().info(f'moving gripper to position: {position}')
        if position > self.max_open or position < self.min_close:
            self.node.get_logger().warn('specified gripper position is outside of gripper range')
            return
        self.gripper.move_to_position(position)
        self.gripper.wait_until_executed()
        time.sleep(1)
    
    def shutdown(self):
        """destroy node"""
        self.executor_thread.join()
        exit(0)


class Gen3LiteArm():
    def __init__(self):
        """initialize MoveIt2 controls for a Kinova Gen 3 Lite arm"""
        self.node = Node("gen3_lite_arm")
        self.callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=['joint_1',
                         'joint_2',
                         'joint_3',
                         'joint_4',
                         'joint_5',
                         'joint_6',
                         'end_effector_link'],
            base_link_name='base_link',
            end_effector_name='end_effector_link',
            group_name='arm',
            callback_group=self.callback_group,
        )
        # subscriber to read current joint angles
        self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        # spin node in background thread and wait for init
        self.executor = rclpy.executors.MultiThreadedExecutor(2)
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        self.executor_thread.start()
        self.node.create_rate(1.0).sleep()
        # set moveit planning parameters
        self.moveit2.pipeline_id = 'ompl'
        self.moveit2.planner_id = 'RRTConnectkConfigDefault'
        self.moveit2.allowed_planning_time = 5.0
        self.moveit2.num_planning_attempts = 10
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1
        self.moveit2.cartesian_jump_threshold = 0.0
        # variable for joint angles
        self.joint_angles = None
   
    def joint_state_callback(self, msg):
        """callback function for updating joint angles"""
        self.joint_state = list(msg.position)
        self.joint_state_timestamp = time.time()
        time.sleep(1)

    def inverse_kinematic_movement(self, target_pose):
        """
        plan and move to a given end effector pose
        target_pose: geometry_msgs Pose
        returns: None
        """
        self.node.get_logger().info(f'moving to pose: {target_pose.position} {target_pose.orientation}')
        self.moveit2.move_to_pose(
            pose=target_pose,
            cartesian=False,
            cartesian_max_step=0.0025,
            cartesian_fraction_threshold=0.0,
        )
        rate = self.node.create_rate(10)
        while self.moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()
        future = self.moveit2.get_execution_future()
        while not future.done():
            rate.sleep()
    
    def forward_kinematic_movement(self, target_angles):
        """
        plan and move to a set of joint angles
        target_angles: list of floats (joints 1-6) of goal angles in radians
        returns: None
        """
        self.node.get_logger().info(f'moving to angles: {target_angles}')
        self.moveit2.move_to_configuration(joint_positions=target_angles)
        rate = self.node.create_rate(10)
        while self.moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()
        future = self.moveit2.get_execution_future()
        while not future.done():
            rate.sleep()
    
    def get_end_effector_pose(self):
        """
        get current pose of the end effector link
        returns: geometry_msgs Pose
        """
        curr_pose_stamped = self.moveit2.compute_fk()
        pose = curr_pose_stamped.pose
        return pose
    
    def get_joint_angles(self):
        """
        get current joint angles in radians
        returns list or None
        """
        if self.joint_state is not None and (time.time() - self.joint_state_timestamp) < 1.0:
            return self.joint_state
    
    def go_home(self):
        """move to home position"""
        self.node.get_logger().info(f'moving to home position')
        self.forward_kinematic_movement([0.0, 0.0, 2.618, 0.0, 0.0, 0.0])
    
    def go_vertical(self):
        """move to vertical position"""
        self.node.get_logger().info(f'moving to vertical position')
        self.forward_kinematic_movement([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def shutdown(self):
        """destroy node"""
        self.executor_thread.join()
        exit(0)


def main():
    rclpy.init()
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()
    pose = Pose()
    pose.position = Point(x=0.077, y=0.094, z=0.799)
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    print(arm.get_end_effector_pose())
    arm.inverse_kinematic_movement(pose)
    print(arm.get_end_effector_pose())
    arm.go_home()
    arm.go_vertical()
    gripper.open()
    gripper.move_to_position(0.5)
    gripper.close()
    rclpy.shutdown()
    gripper.shutdown()
    arm.shutdown()


if __name__ == '__main__':
    main()
