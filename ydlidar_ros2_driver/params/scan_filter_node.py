#!/usr/bin/env python3
"""
scan_filter_node.py
-------------------
Robust multi-stage LaserScan filter:

  Stage 1 – Range bounds
      Drop readings outside [range_min, range_max].

  Stage 2 – One-sided jump filter (strict)
      Remove a point if its range differs from the median of EITHER its
      left OR right window by more than jump_thresh. Catches edge ghost
      points that only have a big jump on one side.

  Stage 3 – Connected-component pruning
      Scan the valid-reading array for consecutive "islands". Any island
      shorter than min_cluster_rays is removed. Wall readings form large
      islands (100s of rays); ghost clusters are small (< 30 rays).
      This is the most robust stage.

Subscribes:  /scan          (raw LaserScan, BEST_EFFORT QoS)
Publishes:   /scan_filtered (cleaned LaserScan, BEST_EFFORT QoS)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np


class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter')

        self.declare_parameter('range_min',        0.15)   # m
        self.declare_parameter('range_max',        10.0)   # m
        self.declare_parameter('jump_window',      8)      # rays each side
        self.declare_parameter('jump_thresh',      0.15)   # m — strict one-sided jump
        self.declare_parameter('min_cluster_rays', 25)     # min consecutive valid rays to keep

        # Chassis deadzone parameters (box around robot footprint)
        self.declare_parameter('chassis_x_min',    -0.45)  # m (rear)
        self.declare_parameter('chassis_x_max',     0.45)  # m (front)
        self.declare_parameter('chassis_y_min',    -0.45)  # m (right)
        self.declare_parameter('chassis_y_max',     0.45)  # m (left)
        self.declare_parameter('chassis_radius',    0.0)   # m (optional radial deadzone)
        self.declare_parameter('lidar_offset_x',    0.0)   # m (lidar X relative to base_link)
        self.declare_parameter('lidar_offset_y',    0.0)   # m (lidar Y relative to base_link)

        self.range_min        = self.get_parameter('range_min').value
        self.range_max        = self.get_parameter('range_max').value
        self.jump_window      = self.get_parameter('jump_window').value
        self.jump_thresh      = self.get_parameter('jump_thresh').value
        self.min_cluster_rays = self.get_parameter('min_cluster_rays').value

        self.chassis_x_min    = self.get_parameter('chassis_x_min').value
        self.chassis_x_max    = self.get_parameter('chassis_x_max').value
        self.chassis_y_min    = self.get_parameter('chassis_y_min').value
        self.chassis_y_max    = self.get_parameter('chassis_y_max').value
        self.chassis_radius   = self.get_parameter('chassis_radius').value
        self.lidar_offset_x   = self.get_parameter('lidar_offset_x').value
        self.lidar_offset_y   = self.get_parameter('lidar_offset_y').value

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            LaserScan, '/scan_filtered', qos_profile_sensor_data)

        self._call_count = 0
        self.get_logger().info(
            f'ScanFilter | range=[{self.range_min},{self.range_max}]m  '
            f'deadzone_x=[{self.chassis_x_min},{self.chassis_x_max}]m  '
            f'deadzone_y=[{self.chassis_y_min},{self.chassis_y_max}]m  '
            f'jump_thresh={self.jump_thresh}m  '
            f'min_cluster={self.min_cluster_rays}rays'
        )

    # ------------------------------------------------------------------
    def callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)

        # === Stage 1: Range bounds ===
        ranges = np.where(
            (ranges >= self.range_min) & (ranges <= self.range_max),
            ranges, np.inf
        )

        # === Stage 2: Robot Chassis Deadzone (Rectangular Box & Optional Radius) ===
        finite_idx = np.where(np.isfinite(ranges))[0]
        if len(finite_idx) > 0:
            angles = msg.angle_min + finite_idx * msg.angle_increment
            r_fin = ranges[finite_idx]
            xs = r_fin * np.cos(angles) + self.lidar_offset_x
            ys = r_fin * np.sin(angles) + self.lidar_offset_y

            in_chassis = (
                (xs >= self.chassis_x_min) & (xs <= self.chassis_x_max) &
                (ys >= self.chassis_y_min) & (ys <= self.chassis_y_max)
            )
            if self.chassis_radius > 0.0:
                in_chassis |= (r_fin < self.chassis_radius)

            deadzone_indices = finite_idx[in_chassis]
            ranges[deadzone_indices] = np.inf

        # === Stage 3: One-sided jump filter ===
        # A point is removed if it jumps by more than jump_thresh from
        # the median of EITHER its left window OR its right window.
        w = self.jump_window
        after_jump = ranges.copy()
        for i in range(n):
            if not np.isfinite(ranges[i]):
                continue

            lo = max(0, i - w)
            hi = min(n, i + w + 1)

            left_valid  = ranges[lo:i][np.isfinite(ranges[lo:i])]
            right_valid = ranges[i+1:hi][np.isfinite(ranges[i+1:hi])]

            pt = float(ranges[i])

            left_jump  = (abs(pt - float(np.median(left_valid)))
                          if len(left_valid) >= 3 else 0.0)
            right_jump = (abs(pt - float(np.median(right_valid)))
                          if len(right_valid) >= 3 else 0.0)

            # One-sided: remove if jump from EITHER side exceeds threshold
            if left_jump > self.jump_thresh or right_jump > self.jump_thresh:
                after_jump[i] = np.inf

        # === Stage 4: Connected-component pruning ===
        # Find runs of consecutive finite values and drop short ones.
        filtered = after_jump.copy()
        i = 0
        while i < n:
            if not np.isfinite(after_jump[i]):
                i += 1
                continue
            # Found start of a cluster — find its end
            j = i
            while j < n and np.isfinite(after_jump[j]):
                j += 1
            cluster_len = j - i
            if cluster_len < self.min_cluster_rays:
                filtered[i:j] = np.inf   # drop small cluster
            i = j

        # === Stats log every 50 scans ===
        self._call_count += 1
        if self._call_count % 50 == 0:
            n_raw  = int(np.sum(np.isfinite(ranges)))
            n_out  = int(np.sum(np.isfinite(filtered)))
            self.get_logger().info(
                f'Stats: {n_raw} pts in → {n_out} out '
                f'({n_raw - n_out} ghost pts removed)'
            )

        # === Publish ===
        out = LaserScan()
        out.header          = msg.header
        out.angle_min       = msg.angle_min
        out.angle_max       = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment  = msg.time_increment
        out.scan_time       = msg.scan_time
        out.range_min       = msg.range_min
        out.range_max       = msg.range_max
        out.ranges          = filtered.tolist()
        out.intensities     = msg.intensities
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
