#!/usr/bin/env python3
"""
save_depth_frame.py
Captures one depth frame and saves BOTH:
  - Raw depth values as CSV (for analysis)
  - A visual PNG heatmap (for quick validation)

Run this AFTER sourcing ROS (or after running restart_camera.sh in the same terminal).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt
import datetime


class DepthSaver(Node):
    def __init__(self):
        super().__init__('depth_saver')
        self.bridge = CvBridge()
        self.latest_depth = None

        self.subscription = self.create_subscription(
            Image,
            '/camera/stereo/image_raw',
            self.depth_callback,
            10
        )
        self.get_logger().info('Depth saver ready. Press Enter to capture (CSV + PNG heatmap).')

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def save_frame(self):
        if self.latest_depth is None:
            self.get_logger().warn('No depth data received yet.')
            return

        # Create timestamp
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        base_name = f'depth_{timestamp}'

        # === 1. Save raw CSV ===
        csv_filename = f'{base_name}.csv'
        np.savetxt(csv_filename, self.latest_depth, fmt='%d', delimiter=',')

        # === 2. Save PNG heatmap ===
        png_filename = f'{base_name}.png'

        valid = self.latest_depth[self.latest_depth > 0]
        if len(valid) > 0:
            min_val = valid.min()
            max_val = valid.max()
            mean_val = valid.mean()
        else:
            min_val = max_val = mean_val = 0

        plt.figure(figsize=(12, 8))
        im = plt.imshow(self.latest_depth, cmap='viridis', aspect='auto')
        cbar = plt.colorbar(im, shrink=0.8)
        cbar.set_label('Depth (mm)', fontsize=12)

        plt.title(f"Depth Map - {base_name}\n"
                  f"Min: {min_val} mm | Max: {max_val} mm | Mean: {mean_val:.1f} mm",
                  fontsize=13, pad=15)
        plt.xlabel("Pixel X (0 = left)")
        plt.ylabel("Pixel Y (0 = top)")

        plt.tight_layout()
        plt.savefig(png_filename, dpi=150, bbox_inches='tight')
        plt.close()

        self.get_logger().info(
            f'Saved {csv_filename} + {png_filename} | '
            f'Shape: {self.latest_depth.shape} | '
            f'Min: {min_val} mm | Max: {max_val} mm | Mean: {mean_val:.1f} mm'
        )


def main():
    rclpy.init()
    node = DepthSaver()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            user_input = input("Press Enter to capture (or 'q' to quit): ")
            if user_input.lower() == 'q':
                break
            node.save_frame()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()