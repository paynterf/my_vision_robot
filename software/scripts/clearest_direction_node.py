#!/usr/bin/env python3
"""
clearest_direction_node.py

Analyzes the bottom half of the depth image and determines the "clearest"
direction using 7 sectors labeled in clock positions (9:00 to 3:00).

Optionally sends simple motion commands to the Teensy over UART so the
4-wheel robot can begin reacting to depth data.

Teensy command-mode characters used (after sending 'C' to enter command mode):
  '4' = 10° CCW (left)
  '6' = 10° CW  (right)
  '5' = Stop

KISS first version – rate-limited, safety threshold, easy to disable.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import serial
import time


class ClearestDirectionNode(Node):
    def __init__(self):
        super().__init__('clearest_direction_node')
        self.bridge = CvBridge()

        # ---------- Serial / motion parameters ----------
        self.declare_parameter('enable_motion', False)   # set True to actually drive
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('min_depth_mm', 400.0)    # only turn if best sector is at least this deep
        self.declare_parameter('command_interval_sec', 0.8)
        self.declare_parameter('deadband_deg', 6.0)      # |turn| smaller than this → treat as center

        self.enable_motion = self.get_parameter('enable_motion').value
        self.min_depth_mm = self.get_parameter('min_depth_mm').value
        self.command_interval = self.get_parameter('command_interval_sec').value
        self.deadband_deg = self.get_parameter('deadband_deg').value

        self.ser = None
        self.last_command_time = 0.0
        self.last_command_sent = None

        if self.enable_motion:
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('baudrate').value
            try:
                self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
                time.sleep(0.3)  # allow Teensy to settle
                # Put Teensy into command mode
                self.ser.write(b'C\n')
                self.ser.flush()
                self.get_logger().info(f'Motion ENABLED – opened {port} @ {baud} and sent "C" (command mode)')
            except Exception as e:
                self.get_logger().error(f'Failed to open serial port {port}: {e}')
                self.get_logger().error('Exiting because enable_motion:=True but serial port is unavailable.')
                raise SystemExit(1)
        else:
            self.get_logger().info('Motion DISABLED (enable_motion:=True to drive the robot)')

        # ---------- Depth analysis parameters ----------
        self.subscription = self.create_subscription(
            Image,
            '/camera/stereo/image_raw',
            self.depth_callback,
            10
        )

        self.num_sectors = 7
        self.image_width = 640
        self.bottom_half_start = 240   # rows 240-479

        self.sector_labels = ['9:00', '10:00', '11:00', '12:00', '1:00', '2:00', '3:00']
        self.sector_width = self.image_width // self.num_sectors

        self.get_logger().info('Clearest Direction Node started (7 sectors, bottom half only)')

    def depth_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_bottom = depth[self.bottom_half_start:, :]

        results = []
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

            results.append({
                'label': self.sector_labels[i],
                'valid_count': int(valid_count),
                'avg_depth': avg_depth
            })

            if avg_depth > best_avg:
                best_avg = avg_depth
                best_sector_idx = i

        # Always print the live table
        #self.print_results(results, best_sector_idx)

        # Optionally send a motion command
        if self.enable_motion and self.ser is not None and best_sector_idx >= 0:
            turn_deg = self.calculate_recommended_turn(best_sector_idx)
            self.maybe_send_command(turn_deg, best_avg)

    def calculate_recommended_turn(self, best_idx):
        """
        Center (12:00) = 0°.
        Positive = turn right (CW), Negative = turn left (CCW).
        """
        angle_per_sector = 72.0 / self.num_sectors   # ~10.3°
        center_sector = 3
        turn_deg = (best_idx - center_sector) * angle_per_sector
        return round(turn_deg)

    def maybe_send_command(self, turn_deg, best_depth_mm):
        now = time.time()
        if (now - self.last_command_time) < self.command_interval:
            return

        # Safety: if the clearest path is still too close, just stop
        if best_depth_mm < self.min_depth_mm:
            cmd = '5'
            reason = f'stop (best depth {best_depth_mm:.0f} mm < {self.min_depth_mm:.0f} mm)'
        elif abs(turn_deg) < self.deadband_deg:
            cmd = '5'
            reason = f'stop (turn {turn_deg}° inside deadband ±{self.deadband_deg}°)'
        elif turn_deg < 0:
            cmd = '4'          # CCW / left
            reason = f'left 10° (recommended {turn_deg}°)'
        else:
            cmd = '6'          # CW / right
            reason = f'right 10° (recommended {turn_deg}°)'

        # Avoid sending the exact same command repeatedly (except stop)
        if cmd == self.last_command_sent and cmd != '5':
            return

        try:
            self.ser.write((cmd + '\n').encode('utf-8'))
            self.ser.flush()
            self.last_command_time = now
            self.last_command_sent = cmd
            self.get_logger().info(f'Sent "{cmd}" → {reason}')
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def print_results(self, results, best_idx):
        print("\n" + "=" * 75)
        print(f"{'Sector':<8} {'Total Valid':>12} {'Avg Depth (mm)':>16} {'Best?':>8} {'Recommend Turn':>16}")
        print("-" * 75)

        for i, r in enumerate(results):
            is_best = "YES" if i == best_idx else ""
            turn_str = ""
            if i == best_idx:
                turn_str = f"{self.calculate_recommended_turn(i)}°"
            print(f"{r['label']:<8} {r['valid_count']:>12} {r['avg_depth']:>16.0f} {is_best:>8} {turn_str:>16}")

        print("=" * 75)

        if best_idx >= 0:
            best_label = results[best_idx]['label']
            best_depth = results[best_idx]['avg_depth']
            turn_deg = self.calculate_recommended_turn(best_idx)
            print(f">>> Best direction: {best_label} | Avg depth: {best_depth:.0f} mm | Recommended turn: {turn_deg}°")
        print()

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(b'5\n')   # stop before exit
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


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