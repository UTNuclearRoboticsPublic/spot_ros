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
    '''
    A multithreaded executor is required for the MutuallyExclusiveCallbackGroups
    assigned in SpotROS.connect() to have any effect. It also allows the Rate
    timer inside the WalkTo action callback to be serviced (with the default
    single-threaded executor, Rate.sleep() inside a callback deadlocks the node),
    and prevents a single slow/blocked RPC from starving cmd_vel and the
    estop services.
    '''
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        rclpy.spin(body_node, executor=executor)
    except KeyboardInterrupt:
        pass

    body_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()