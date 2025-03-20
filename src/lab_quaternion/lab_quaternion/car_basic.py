import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)  # 等待发布器初始化

    def move(self, linear_speed=0.15, angular_speed=0.02, duration=22.0):
        """让机器人直线行驶，同时微调角速度防止偏移"""
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed  # 轻微左转修正

        self.publisher_.publish(twist)
        time.sleep(duration)

        # 停止机器人
        self.stop()

    def stop(self):
        """停止机器人（速度归零）。"""
        twist = Twist()
        self.publisher_.publish(twist)
        time.sleep(1)

    def execute_sequence(self):
        """机器人直线到达起点并返回原位置"""
        self.get_logger().info("前进到起点...")
        self.move(0.20, 0.00995, 15.9)  # 增加微小角速度修正右偏

        self.get_logger().info("返回原位置...")
        self.move(-0.20, -0.00995, 15.9)  # 反向运动时同样修正角速度

        self.get_logger().info("运动完成。")

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    controller.execute_sequence()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

