#!/usr/bin/env python3

import rclpy
from spot_driver.spot_ros import SpotROS

def main():
    rclpy.init()

    node = SpotROS()

    if not node.connect():
        return

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()