import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)  # Allow publisher to initialize

    def move(self, linear_speed=0.0, angular_speed=0.0, duration=0.0):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        
        self.publisher_.publish(twist)
        time.sleep(duration)
        
        # Stop the robot after motion
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)

    def execute_sequence(self):

        self.move(linear_speed=0.12, duration=0.2)

        # turn right 25 degree
        radian = 25 * 3.14 / 180
        self.move(angular_speed=-radian, duration=1.0)
        
        self.move(linear_speed=0.12, duration=9.5)

        # turn left 65 degree
        radian = 80 * 3.14 / 180
        self.move(angular_speed=radian, duration=1.0)
        
        self.move(linear_speed=0.12, duration=14)

        # turn right 70 degree
        radian = 92 * 3.14 / 180
        self.move(angular_speed=-radian, duration=1.0)
        
        self.move(linear_speed=0.12, duration=11.2)
        
        self.get_logger().info("Motion sequence completed.")


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    controller.execute_sequence()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
