import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)  # 等待发布器初始化

    def move(self, linear_speed=0.15, angular_speed=0.0, duration=16.0):
        """让机器人以给定速度直线行驶"""
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed  # 不再修正角速度

        self.publisher_.publish(twist)
        time.sleep(duration)

        # 停止机器人
        self.stop()

    def stop(self):
        """停止机器人（速度归零）"""
        twist = Twist()
        self.publisher_.publish(twist)
        self.get_logger().info("机器人已停止。")
        time.sleep(1)

    def rotate_left_degrees(self, duration=0.5):
        """让机器人向左旋转"""
        left_turn_speed = math.radians(3) / duration  # 计算合适的角速度
        self.get_logger().info("开始左转...")
        self.move(0.0, left_turn_speed, duration)
        self.get_logger().info("左转完成！")

    def execute_sequence(self):
        self.get_logger().info("开始执行...")
        
        # 左转
        self.rotate_left_degrees(duration=0.5)

        # 前进 16 秒
        self.get_logger().info("前进 16 秒...")
        self.move(0.20, 0.0, 16.0)  

        # 停止 1 秒
        self.stop()
        
        self.rotate_left_degrees(duration=0.5)
        
        # 后退 16 秒
        self.get_logger().info("后退 16 秒...")
        self.move(-0.20, 0.0, 16.0)

        self.get_logger().info("运动完成！")

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    
    try:
        controller.execute_sequence()
    except KeyboardInterrupt:
        controller.get_logger().info("检测到 Ctrl+C，正在停止机器人...")
        controller.stop()  # 立即停止机器人
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

