import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import signal

class MoveNode(Node):
    movement_speed = 0.1
    movement_duration = 3.0

    def __init__(self):
        super().__init__('move_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(self.movement_duration, self.move_loop)
        signal.signal(signal.SIGINT, self.stop_robot)
        self.forward = True

    def move_loop(self):
        msg = Twist()
        msg.linear.x = self.movement_speed if self.forward else -self.movement_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Moving: {"Forward" if self.forward else "Backward"}')
        self.forward = not self.forward

    def stop_robot(self, signum, frame):
        self.get_logger().info('Stopping robot...')
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        self.publisher_.publish(stop_msg)

        rclpy.shutdown()

def main():
    rclpy.init()
    node = MoveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()