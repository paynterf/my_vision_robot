#!/usr/bin/env python3
"""
clearest_direction_node.py
Analyzes the bottom half of the depth image and publishes the clearest direction.
Does NOT talk to the Teensy – that is WallE_5.py’s job.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

#from my_vision_robot.msg import ClearestDirection   # <-- adjust package name if needed
from my_vision_robot_msgs.msg import ClearestDirection #now points to 


class ClearestDirectionNode(Node):

    def __init__(self):
        super().__init__('clearest_direction_node')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('min_depth_mm', 400.0)
        self.declare_parameter('deadband_deg', 6.0)

        self.min_depth_mm = self.get_parameter('min_depth_mm').value
        self.deadband_deg = self.get_parameter('deadband_deg').value

        # Publisher
        self.publisher = self.create_publisher(ClearestDirection, '/clearest_direction', 10)

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/stereo/image_raw',
            self.depth_callback,
            10
        )

        # Sector setup
        self.num_sectors = 7
        self.image_width = 640
        self.bottom_half_start = 240
        self.sector_labels = ['9:00', '10:00', '11:00', '12:00', '1:00', '2:00', '3:00']
        self.sector_width = self.image_width // self.num_sectors

        self.get_logger().info('Clearest Direction Node started – publishing on /clearest_direction')

    def depth_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_bottom = depth[self.bottom_half_start:, :]

        best_sector_idx = -1
        best_avg = -1.0

        for i in range(self.num_sectors):
            start_col = i * self.sector_width
            end_col = (i + 1) * self.sector_width if i < self.num_sectors - 1 else self.image_width
            sector_data = depth_bottom[:, start_col:end_col]

            valid_mask = sector_data > 0
            valid_count = np.sum(valid_mask)

            if valid_count > 0:
                avg_depth = float(np.mean(sector_data[valid_mask]))
            else:
                avg_depth = 0.0

            if avg_depth > best_avg:
                best_avg = avg_depth
                best_sector_idx = i

        # Build and publish the message
        out = ClearestDirection()
        out.header = msg.header
        out.best_sector = best_sector_idx if best_sector_idx >= 0 else 0
        out.best_depth_mm = best_avg if best_sector_idx >= 0 else 0.0
        out.turn_deg = float(self.calculate_recommended_turn(best_sector_idx)) if best_sector_idx >= 0 else 0.0

        if best_sector_idx < 0 or best_avg <= 0.0:
            out.status = 2          # NO_VALID_DATA
        elif best_avg < self.min_depth_mm:
            out.status = 1          # TOO_CLOSE
        else:
            out.status = 0          # OK

        self.publisher.publish(out)

        # Optional debug
        # self.get_logger().info(
        #     f'Sector {self.sector_labels[best_sector_idx]} | '
        #     f'depth {best_avg:.0f} mm | turn {out.turn_deg:.1f}° | status {out.status}'
        # )

    def calculate_recommended_turn(self, best_idx):
        """Center (12:00) = 0°. Positive = CW (right), Negative = CCW (left)."""
        if best_idx < 0:
            return 0.0
        angle_per_sector = 72.0 / self.num_sectors   # ≈ 10.3°
        center_sector = 3
        return round((best_idx - center_sector) * angle_per_sector)


def main():
    rclpy.init()
    node = ClearestDirectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()