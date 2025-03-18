#!/usr/bin/python3
import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster

def main():
    rclpy.init()

    node = Node("slam_inital_pose_frame_publisher")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    tf_broadcaster = StaticTransformBroadcaster(node)

    intial_pose_frame = 'base_footprint'
    odom_frame = 'odom'

    def publish_transform():
        nonlocal timer
        try:
            transform = tf_buffer.lookup_transform(intial_pose_frame, odom_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=5.0))
        except Exception as e:
            node.get_logger().error(f'Unable to publish "slam_initial_pose" transform: {e}')
            rclpy.shutdown()
            exit(0)

        transform.header.frame_id = 'slam_initial_pose'
        transform.child_frame_id = odom_frame
        tf_broadcaster.sendTransform(transform)

        node.get_logger().info('Initial transform published')
        timer.destroy()

    timer = node.create_timer(0.5, publish_transform)

    rclpy.spin(node)

if __name__ == '__main__':
    main()