import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveNode(Node):
    movement_speed = 0.2
    movement_duration = 5.0

    def __init__(self):
        super().__init__('move_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(self.movement_duration, self.move_loop)
        self.forward = True

    def move_loop(self):
        msg = Twist()

        if self.forward:
            msg.linear.x = self.movement_speed
            self.get_logger().info("Moving forward")
        else:
            msg.linear.x = -self.movement_speed
            self.get_logger().info("Moving backward")

        self.publisher_.publish(msg)
        self.forward = not self.forward

def main():
    rclpy.init()  
    node = MoveNode()

    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        node.get_logger().warn("Keyboard interrupt detected")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()