#!/usr/bin/env python3
"""
clearest_direction_node.py

Analyzes the bottom half of the depth image and determines the "clearest"
direction using 7 sectors labeled in clock positions (9:00 to 3:00).

For now it only prints to the console (KISS principle).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class ClearestDirectionNode(Node):
    def __init__(self):
        super().__init__('clearest_direction_node')
        self.bridge = CvBridge()

        # Subscribe to depth image
        self.subscription = self.create_subscription(
            Image,
            '/camera/stereo/image_raw',
            self.depth_callback,
            10
        )

        # Parameters
        self.num_sectors = 7
        self.image_width = 640
        self.bottom_half_start = 240   # Only use rows 240-479 (bottom half)

        # Sector labels (clock positions)
        self.sector_labels = ['9:00', '10:00', '11:00', '12:00', '1:00', '2:00', '3:00']

        # Sector width in pixels
        self.sector_width = self.image_width // self.num_sectors

        self.get_logger().info('Clearest Direction Node started (7 sectors, bottom half only)')

    def depth_callback(self, msg):
        # Convert to numpy array
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Only use bottom half of the image
        depth_bottom = depth[self.bottom_half_start:, :]

        results = []
        best_sector_idx = -1
        best_avg = -1.0

        for i in range(self.num_sectors):
            start_col = i * self.sector_width
            end_col = (i + 1) * self.sector_width if i < self.num_sectors - 1 else self.image_width

            sector_data = depth_bottom[:, start_col:end_col]

            # Valid (non-zero) pixels
            valid_mask = sector_data > 0
            valid_count = np.sum(valid_mask)

            if valid_count > 0:
                avg_depth = np.mean(sector_data[valid_mask])
            else:
                avg_depth = 0.0

            results.append({
                'label': self.sector_labels[i],
                'valid_count': valid_count,
                'avg_depth': avg_depth
            })

            if avg_depth > best_avg:
                best_avg = avg_depth
                best_sector_idx = i

        # Print results in tabular format
        self.print_results(results, best_sector_idx)

    def print_results(self, results, best_idx):
        print("\n" + "=" * 75)
        print(f"{'Sector':<8} {'Total Valid':>12} {'Avg Depth (mm)':>16} {'Best?':>8} {'Recommend Turn':>16}")
        print("-" * 75)

        for i, r in enumerate(results):
            is_best = "YES" if i == best_idx else ""
            turn_str = self.calculate_recommended_turn(i, best_idx) if i == best_idx else ""
            print(f"{r['label']:<8} {r['valid_count']:>12} {r['avg_depth']:>16.0f} {is_best:>8} {turn_str:>16}")

        print("=" * 75)

        if best_idx >= 0:
            best_label = results[best_idx]['label']
            best_depth = results[best_idx]['avg_depth']
            turn_deg = self.calculate_recommended_turn(best_idx)
            print(f">>> Best direction: {best_label} | Avg depth: {best_depth:.0f} mm | Recommended turn: {turn_deg}°")
        print()

    def calculate_recommended_turn(self, best_idx, reference_idx=None):
        """
        Simple angle estimation based on sector position.
        Center (12:00) = 0°. Positive = turn right (CW), Negative = turn left (CCW).
        """
        # Approximate angle per sector (72° HFOV / 7 sectors ≈ 10.3°)
        angle_per_sector = 72.0 / self.num_sectors

        # Sector 3 (12:00) is center
        center_sector = 3
        turn_deg = (best_idx - center_sector) * angle_per_sector

        return round(turn_deg)


def main():
    rclpy.init()
    node = ClearestDirectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()