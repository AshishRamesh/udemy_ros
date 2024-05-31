#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.action import CountUntil

class CountUntilServerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("count_until_server") # MODIFY NAME


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()