#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

class FrameListener(Node):

    def __init__(self):
        super().__init__('tag_tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(TransformStamped, '/tag0_tf', 10)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
    def on_timer(self):
        to_frame_rel = 'base_link'
        from_frame_rel = 'tag16h5:0_rotated'
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'tag16h5:0_rotated',
                now)
            self.publisher.publish(trans)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()