#!/usr/bin/env python3

import rclpy
from spot_driver.spot_ros import SpotROS
from spot_driver.spot_base_wrapper import SpotBaseWrapper

def main():
    rclpy.init()

    base_wrapper = SpotBaseWrapper()
    body_node = SpotROS()

    if not body_node.connect(base_wrapper):
        return

    rclpy.spin(body_node)

    body_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()