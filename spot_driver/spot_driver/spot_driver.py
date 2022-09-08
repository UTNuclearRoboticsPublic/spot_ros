#!/usr/bin/env python3

from spot_driver.spot_ros import SpotROS

if __name__ == '__main__':
    node = SpotROS()
    node.main()