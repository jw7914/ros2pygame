import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pygame_test.button_map import ButtonMap
import time

class JoyTester(Node):
    def __init__(self):
        super().__init__('joy_tester')
        self.get_logger().info('Testing Joystick...')
        
        # Subscribe to the 'joy' topic
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Timestamp for rate limiting
        self.last_processed_time = time.time()
        self.rate_limit = 1/3  # 2 times per second (1/2 = 0.5 seconds)

    def joy_callback(self, joy_msg):
        current_time = time.time()
        # Check if enough time has passed since the last processing
        if current_time - self.last_processed_time >= self.rate_limit:
            self.last_processed_time = current_time  # Update the timestamp
            
            # Process the button inputs using ButtonMap
            if joy_msg.buttons[ButtonMap.A.value] == 1:
                self.get_logger().info("Button A pressed")

            if joy_msg.buttons[ButtonMap.B.value] == 1:
                self.get_logger().info("Button B pressed")

            if joy_msg.buttons[ButtonMap.Y.value] == 1:
                self.get_logger().info("Button Y pressed")

            if joy_msg.buttons[ButtonMap.X.value] == 1:
                self.get_logger().info("Button X pressed")

            if joy_msg.buttons[ButtonMap.LB.value] == 1:
                self.get_logger().info("Left Trigger pressed")
            
            if joy_msg.buttons[ButtonMap.RB.value] == 1:
                self.get_logger().info("Right Trigger pressed")

            if joy_msg.buttons[ButtonMap.WINDOW.value] == 1:
                self.get_logger().info("Window Button pressed")

            if joy_msg.buttons[ButtonMap.MENU.value] == 1:
                self.get_logger().info("Menu Button pressed")

            if joy_msg.buttons[ButtonMap.XBOX.value] == 1:
                self.get_logger().info("Xbox button pressed")

            if joy_msg.buttons[ButtonMap.LEFT_ANALOG.value] == 1:
                self.get_logger().info("Left Analog pressed")

            if joy_msg.buttons[ButtonMap.RIGHT_ANALOG.value] == 1:
                self.get_logger().info("Right Analog pressed")

            if joy_msg.buttons[ButtonMap.EXIT.value] == 1:
                self.get_logger().info("Exit Button pressed")
                
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