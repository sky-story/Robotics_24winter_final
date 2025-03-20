#!/usr/bin/env python3
import time
import rclpy
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion as PyQuaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper

def move_to_vertical(arm):
    """
    让机械臂回到竖直位置
    """
    print("[DEBUG] 开始移动到竖直位置...")
    arm.go_vertical()
    time.sleep(1)
    print("[DEBUG] 已到达竖直位置")

def move_to_pose(arm, pose, description="目标位置"):
    """
    让机械臂移动到指定位置
    """
    print(f"[DEBUG] 开始移动到 {description}...")
    arm.inverse_kinematic_movement(pose)
    print(f"[DEBUG] 已到达 {description} (x={pose.position.x}, y={pose.position.y}, z={pose.position.z})")
    time.sleep(0.5)

def pick_and_place(arm, gripper):
    """
    机械臂执行 pick-and-place 任务（包含 4 个控制点）
    """
    print("[DEBUG] 机械臂开始执行 pick-and-place 任务...")
    
    pre_pick_pose = Pose()
    pre_pick_pose.position.x = 0.388
    pre_pick_pose.position.y = -0.06
    pre_pick_pose.position.z = 0.241
    pre_pick_pose.orientation.x = -0.694
    pre_pick_pose.orientation.y = 0.718
    pre_pick_pose.orientation.z = -0.044
    pre_pick_pose.orientation.w = -0.022
    
    pick_pose = Pose()
    pick_pose.position.x = 0.380
    pick_pose.position.y = -0.025
    pick_pose.position.z = 0.143
    pick_pose.orientation.x = -0.695
    pick_pose.orientation.y = 0.718
    pick_pose.orientation.z = -0.044
    pick_pose.orientation.w = -0.022
    
    pre_put_pose = Pose()
    pre_put_pose.position.x = -0.199
    pre_put_pose.position.y = 0.014
    pre_put_pose.position.z = -0.242  # 预备放置点（高于 put_pose）
    pre_put_pose.orientation.x = 0.999
    pre_put_pose.orientation.y = 0.012
    pre_put_pose.orientation.z = -0.011
    pre_put_pose.orientation.w = 0.048
    
    put_pose = Pose()
    put_pose.position.x = -0.186
    put_pose.position.y = 0.019
    put_pose.position.z = -0.324
    put_pose.orientation.x = 1.000
    put_pose.orientation.y = 0.011
    put_pose.orientation.z = -0.011
    put_pose.orientation.w = 0.006
    
    # 1. 夹爪张开
    print("[DEBUG] 夹爪张开...")
    gripper.move_to_position(0.0)
    time.sleep(0.5)
    
    # 2. 先移动到预备拾取位置，再下降到拾取位置
    move_to_pose(arm, pre_pick_pose, "预备拾取位置")
    move_to_pose(arm, pick_pose, "拾取位置")
    
    # 3. 夹取物体
    print("[DEBUG] 夹取物体...")
    gripper.move_to_position(0.7)
    time.sleep(0.5)
    
    # 4. 先移动到预备放置位置，再下降到放置位置
    move_to_pose(arm, pre_put_pose, "预备放置位置")
    move_to_pose(arm, put_pose, "放置位置")
    
    # 5. 释放物体
    print("[DEBUG] 释放物体...")
    gripper.move_to_position(0.0)
    time.sleep(0.5)
    
    print("[DEBUG] 任务完成！")

def main():
    rclpy.init()
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()
    
    try:
        # move_to_vertical(arm)  # 先让机械臂回到竖直位置
        pick_and_place(arm, gripper)
    finally:
        print("[DEBUG] 关闭 ROS 2 节点...")
        rclpy.shutdown()
        gripper.shutdown()
        arm.shutdown()

if __name__ == '__main__':
    main()
