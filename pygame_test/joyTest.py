import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyTester(Node):
    def __init__(self):
        super().__init__('joy_tester')
        self.get_logger().info('Testing Joystick...')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.buttons = []
        self.axes = []

    def joy_callback(self, joy_msg):
        for i, val in enumerate(joy_msg.buttons):
            self.buttons[i].update_value(val)
            print(val)


        for i, val in enumerate(joy_msg.axes):
            self.axes[i].update_value(val)
            print(val)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    joy_tester = JoyTester()
    try:
        joy_tester.run()
    finally:
        joy_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
