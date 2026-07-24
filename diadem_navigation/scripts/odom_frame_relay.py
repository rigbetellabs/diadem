#!/usr/bin/python3
"""
odom_frame_relay.py
-------------------
Re-publishes FAST-LIO2's odometry with corrected frame IDs so the EKF
(robot_localization) can consume it without TF tree conflicts.

FAST-LIO2 hardcodes:
  header.frame_id  : camera_init  -->  odom
  child_frame_id   : body         -->  base_link

Pipeline:
  FAST-LIO /fast_lio/odometry (camera_init/body)
      --> this relay
      --> /fast_lio/odometry_corrected (odom/base_link)
      --> EKF (fuses with GPS + IMU)
      --> /odometry/filtered + odom->base_link TF
      --> Nav2
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from copy import deepcopy


class OdomFrameRelay(Node):
    def __init__(self):
        super().__init__('odom_frame_relay')

        self.declare_parameter('input_topic',  '/fast_lio/odometry')
        self.declare_parameter('output_topic', '/fast_lio/odometry_corrected')
        self.declare_parameter('src_frame',    'camera_init')
        self.declare_parameter('src_child',    'body')
        self.declare_parameter('dst_frame',    'odom')
        self.declare_parameter('dst_child',    'base_link')

        in_topic        = self.get_parameter('input_topic').value
        out_topic       = self.get_parameter('output_topic').value
        self.src_frame  = self.get_parameter('src_frame').value
        self.src_child  = self.get_parameter('src_child').value
        self.dst_frame  = self.get_parameter('dst_frame').value
        self.dst_child  = self.get_parameter('dst_child').value

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.sub = self.create_subscription(Odometry, in_topic, self._cb, 10)

        self.gps_pub = self.create_publisher(Odometry, '/odometry/gps_corrected', 10)
        self.gps_sub = self.create_subscription(Odometry, '/odometry/gps', self._gps_cb, 10)
        self.gps_fix_sub = self.create_subscription(NavSatFix, '/gps/fix', self._gps_fix_cb, 10)

        # Health monitoring state variables
        self.last_lio_time = self.get_clock().now()
        self.last_vio_time = self.get_clock().now()
        self.last_vio_covariance = 9999.0
        self.gps_state = 'suppressed'  # 'suppressed' or 'active'

        # Subscribe to VIO to monitor its tracking health
        self.vio_sub = self.create_subscription(Odometry, '/vio/odom', self._vio_cb, 10)

        self.get_logger().info(
            f'Relaying {in_topic} ({self.src_frame}->{self.src_child}) '
            f'to {out_topic} ({self.dst_frame}->{self.dst_child})'
        )

    def _cb(self, msg: Odometry):
        out = deepcopy(msg)

        # Fix FAST-LIO's hardcoded non-standard frame names
        if out.header.frame_id == self.src_frame:
            out.header.frame_id = self.dst_frame
        if out.child_frame_id == self.src_child:
            out.child_frame_id = self.dst_child

        # Zero Z — 2D navigation; also guards against IMU vertical drift
        # when LiDAR has no valid scan (avoids 2453m altitude bug)
        out.pose.pose.position.z = 0.0
        out.twist.twist.linear.z = 0.0

        # Velocity deadband: zero out tiny twist values from Gazebo physics settling.
        # These micro-velocities (< 5mm/s) at startup get integrated by the EKF into drift.
        DEADBAND = 0.005  # m/s
        if abs(out.twist.twist.linear.x) < DEADBAND:
            out.twist.twist.linear.x = 0.0
        if abs(out.twist.twist.linear.y) < DEADBAND:
            out.twist.twist.linear.y = 0.0
        if abs(out.twist.twist.angular.z) < DEADBAND:
            out.twist.twist.angular.z = 0.0

        # Record FAST-LIO update timestamp as healthy local input
        self.last_lio_time = self.get_clock().now()

        self.pub.publish(out)

    def _vio_cb(self, msg: Odometry):
        # Record VIO update timestamp and covariance
        self.last_vio_time = self.get_clock().now()
        self.last_vio_covariance = msg.pose.covariance[0]

    def _gps_cb(self, msg: Odometry):
        out = deepcopy(msg)

        current_time = self.get_clock().now()
        # Evaluate local dead-reckoning health (LIO or VIO active)
        lio_healthy = (current_time - self.last_lio_time).nanoseconds / 1e9 < 1.5
        vio_healthy = (current_time - self.last_vio_time).nanoseconds / 1e9 < 1.5 and self.last_vio_covariance < 10.0

        if lio_healthy or vio_healthy:
            # Local navigation is healthy: Suppress GPS to prevent goal drift and jitter.
            # 1,000,000.0 variance (1000m std dev) means EKF ignores GPS corrections.
            out.pose.covariance[0] = 1000000.0
            out.pose.covariance[7] = 1000000.0
            if self.gps_state != 'suppressed':
                self.get_logger().info("FAST-LIO/VIO local odometry is healthy. Suppressing GPS fusion in Global EKF (covariance = 1e6).")
                self.gps_state = 'suppressed'
        else:
            # Both local sensors failed: Fall back to GPS to bound dead-reckoning drift.
            # 4.0 variance (2m std dev) gives EKF a stable target to pull the robot back to.
            out.pose.covariance[0] = 4.0
            out.pose.covariance[7] = 4.0
            if self.gps_state != 'active':
                self.get_logger().error("Both FAST-LIO and VIO are FAILING! Enabling GPS fusion in Global EKF (covariance = 4.0).")
                self.gps_state = 'active'

        self.gps_pub.publish(out)

    def _gps_fix_cb(self, msg: NavSatFix):
        # Log GPS coordinates at 2-second intervals to avoid flooding the log
        self.get_logger().info(
            f"Raw GPS Coordinates -> Latitude: {msg.latitude:.7f}, Longitude: {msg.longitude:.7f}, Altitude: {msg.altitude:.3f}m",
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
