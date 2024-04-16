#!/usr/bin/env python3

import rclpy
from spot_driver.spot_ros import SpotROS
from spot_driver.spot_lease_manager import SpotLeaseManager

def main():
    rclpy.init()

    lease_manager = SpotLeaseManager()
    body_node = SpotROS()

    if not body_node.connect(lease_manager):
        return

    rclpy.spin(body_node)

    body_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()