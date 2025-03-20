import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2Interface
from geometry_msgs.msg import PoseStamped
import time

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        self.moveit2 = MoveIt2Interface()
        
    def move_to(self, x, y, z, qx, qy, qz, qw):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.moveit2.move_to_pose(pose)
        self.moveit2.wait_for_motion()

    def pick_cube(self, cube_pose):
        # 移动到上方
        self.move_to(cube_pose[0], cube_pose[1], cube_pose[2] + 0.1, cube_pose[3], cube_pose[4], cube_pose[5], cube_pose[6])
        time.sleep(1)
        # 下降
        self.move_to(cube_pose[0], cube_pose[1], cube_pose[2], cube_pose[3], cube_pose[4], cube_pose[5], cube_pose[6])
        time.sleep(1)
        # 夹爪闭合
        self.moveit2.close_gripper()
        time.sleep(1)
        # 抬起
        self.move_to(cube_pose[0], cube_pose[1], cube_pose[2] + 0.1, cube_pose[3], cube_pose[4], cube_pose[5], cube_pose[6])
    
    def place_cube(self, place_pose):
        # 移动到放置点
        self.move_to(place_pose[0], place_pose[1], place_pose[2] + 0.1, place_pose[3], place_pose[4], place_pose[5], place_pose[6])
        time.sleep(1)
        # 下降
        self.move_to(place_pose[0], place_pose[1], place_pose[2], place_pose[3], place_pose[4], place_pose[5], place_pose[6])
        time.sleep(1)
        # 释放
        self.moveit2.open_gripper()
        time.sleep(1)
        # 抬起
        self.move_to(place_pose[0], place_pose[1], place_pose[2] + 0.1, place_pose[3], place_pose[4], place_pose[5], place_pose[6])

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()

    # 立方体和目标位置的位姿 (x, y, z, qx, qy, qz, qw)
    green_cube_pose = [0.4, 0.1, 0.05, 0, 0, 0, 1]
    green_bin_pose = [0.6, 0.2, 0.05, 0, 0, 0, 1]
    
    node.pick_cube(green_cube_pose)
    node.place_cube(green_bin_pose)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
