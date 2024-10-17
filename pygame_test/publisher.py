#!/home/jason/Desktop/ros2ws/src/pygame_test/.venv/bin/python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame

class PublishInputs(Node):
    def __init__(self):
        super().__init__("keyboard_publisher")
        self.cmd_publisher = self.create_publisher(String, 'keyboard_commands', 10)
        self.get_logger().info("Keyboard input node started")

    def send_cmd(self, key):
        msg = String()
        msg.data = key
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"Published: {key}")

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Initialize Pygame
    pygame.init()

    # Create a minimal display
    screen = pygame.display.set_mode((1, 1))  # Minimal size to allow for event handling
    pygame.display.set_caption("Keyboard Listener")

    node = PublishInputs()

    # Run until the user asks to quit
    running = True
    while running:
        for event in pygame.event.get():
            # Check for quit event
            if event.type == pygame.QUIT:
                running = False

            # Check for keydown event
            if event.type == pygame.KEYDOWN:
                key_name = pygame.key.name(event.key)
                node.send_cmd(key_name)

        # Add a short delay Optional
        pygame.time.delay(10)

    pygame.quit()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
