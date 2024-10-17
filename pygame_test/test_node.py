#!usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame

class MyNode(Node):
    def __init__(self):
        super().__init__("firstnode")
        self.counter = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter))
        self.counter += 1  # Increment the counter instead of resetting to 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    # Keep spinning until the node is shut down
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
