#!/home/jason/Desktop/ros2ws/src/pygame_test/.venv/bin/python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardCommandSubscriber(Node):
    def __init__(self):
        super().__init__('keyboard_subscriber')
        self.subscription = self.create_subscription(
            String,
            'keyboard_commands',
            self.listener_callback,
            10)
        self.get_logger().info("Listening for keyboard commands...")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCommandSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
