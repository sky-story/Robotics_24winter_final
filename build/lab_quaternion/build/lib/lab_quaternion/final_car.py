import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)  # Allow publisher to initialize

    def move(self, linear_speed=0.0, angular_speed=0.0, duration=0.0):
        """Publishes velocity commands to move the robot."""
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        
        self.publisher_.publish(twist)
        time.sleep(duration)
        
        # Stop the robot after motion
        self.stop()

    def stop(self):
        """Stops the robot by publishing zero velocity."""
        twist = Twist()
        self.publisher_.publish(twist)
        time.sleep(1)

    def execute_sequence(self):
        """Executes a predefined movement sequence."""
        movements = [
            (0.12, 0.0, 0.2),  # Move forward
            (0.0, -math.radians(30), 1.0),  # Turn right 30 degrees
            (0.12, 0.0, 10.0),  # Move forward
            (0.0, math.radians(85), 1.0),  # Turn left 85 degrees
            (0.12, 0.0, 14.7),  # Move forward
            (0.0, -math.radians(90), 1.0),  # Turn right 90 degrees
            (0.12, 0.0, 10.3)  # Move forward
        ]
        
        for linear, angular, duration in movements:
            self.move(linear, angular, duration)
        
        self.get_logger().info("Motion sequence completed.")


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    controller.execute_sequence()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
