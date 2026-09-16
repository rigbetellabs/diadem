#!/usr/bin/env python3
"""
Comprehensive Nav2 Goal & Yaw Diagnostic Tool
---------------------------------------------
Tracks:
  - Current Robot Pose (in map and odom)
  - Goal Pose (from /goal_pose or /plan)
  - XY Distance to Goal vs Tolerance
  - Yaw Error vs Tolerance
  - Commanded Velocities (wz, vx)

Reveals precisely why navigation fails to complete (XY error vs Yaw error vs Controller Hunting).
"""

import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import tf2_ros


def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angular_distance(from_angle, to_angle):
    return math.atan2(math.sin(to_angle - from_angle), math.cos(to_angle - from_angle))


class GoalDiagnosticNode(Node):
    def __init__(self):
        super().__init__('goal_diagnostic_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_wz = 0.0
        self.latest_vx = 0.0
        self.last_wz_sign = 0

        self.goal_x = None
        self.goal_y = None
        self.goal_yaw_rad = None
        self.goal_frame = 'map'

        self.prev_odom_yaw = None
        self.prev_time = self.get_clock().now()

        # Tolerances from nav2_params.yaml
        self.xy_tol = 0.25
        self.yaw_tol_rad = 0.30  # ~17.2 deg

        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.sub_plan = self.create_subscription(
            Path, '/plan', self.plan_callback, 10
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.sub_cmd_vel_smoothed = self.create_subscription(
            Twist, '/cmd_vel_smoothed', self.cmd_vel_smoothed_callback, 10
        )

        self.timer = self.create_timer(0.1, self.tick)
        self.flip_count = 0

        self.get_logger().info("================================================================")
        self.get_logger().info(" Nav2 Goal & Controller Diagnostic (10 Hz)")
        self.get_logger().info(" Subscribing to /goal_pose, /plan, /tf, /cmd_vel")
        self.get_logger().info(f" Checking tolerances: XY <= {self.xy_tol:.2f}m, YAW <= {math.degrees(self.yaw_tol_rad):.1f}°")
        self.get_logger().info("================================================================")
        print(f"{'TIME':<8} | {'ROBOT YAW':<9} | {'GOAL YAW':<9} | {'YAW ERR':<9} | {'XY DIST':<8} | {'XY OK?':<6} | {'YAW OK?':<7} | {'CMD wz':<10} | {'STATUS'}")
        print("-" * 95)

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        q = msg.pose.orientation
        self.goal_yaw_rad = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.goal_frame = msg.header.frame_id or 'map'
        self.get_logger().info(f"--> New Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) Yaw: {math.degrees(self.goal_yaw_rad):.1f}° in frame '{self.goal_frame}'")

    def plan_callback(self, msg: Path):
        if msg.poses:
            last = msg.poses[-1]
            self.goal_x = last.pose.position.x
            self.goal_y = last.pose.position.y
            q = last.pose.orientation
            self.goal_yaw_rad = euler_from_quaternion(q.x, q.y, q.z, q.w)
            self.goal_frame = last.header.frame_id or 'map'

    def cmd_vel_callback(self, msg: Twist):
        self.latest_vx = msg.linear.x
        self.latest_wz = msg.angular.z

    def cmd_vel_smoothed_callback(self, msg: Twist):
        self.latest_vx = msg.linear.x
        self.latest_wz = msg.angular.z

    def tick(self):
        now = self.get_clock().now()
        t_sec = now.nanoseconds / 1e9

        robot_x, robot_y, robot_yaw_rad = None, None, None
        try:
            t = self.tf_buffer.lookup_transform(
                self.goal_frame, 'base_link', rclpy.time.Time()
            )
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
            q = t.transform.rotation
            robot_yaw_rad = euler_from_quaternion(q.x, q.y, q.z, q.w)
        except Exception:
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time()
                )
                robot_x = t.transform.translation.x
                robot_y = t.transform.translation.y
                q = t.transform.rotation
                robot_yaw_rad = euler_from_quaternion(q.x, q.y, q.z, q.w)
            except Exception:
                pass

        robot_yaw_str = f"{math.degrees(robot_yaw_rad):+.1f}°" if robot_yaw_rad is not None else "N/A"
        goal_yaw_str = f"{math.degrees(self.goal_yaw_rad):+.1f}°" if self.goal_yaw_rad is not None else "WAITING"

        xy_dist = None
        xy_ok = False
        if robot_x is not None and self.goal_x is not None:
            xy_dist = math.hypot(robot_x - self.goal_x, robot_y - self.goal_y)
            xy_ok = xy_dist <= self.xy_tol

        yaw_err_deg = None
        yaw_ok = False
        if robot_yaw_rad is not None and self.goal_yaw_rad is not None:
            yaw_err_rad = shortest_angular_distance(robot_yaw_rad, self.goal_yaw_rad)
            yaw_err_deg = math.degrees(yaw_err_rad)
            yaw_ok = abs(yaw_err_rad) <= self.yaw_tol_rad

        # Detect direction flips
        current_wz = self.latest_wz
        current_sign = 1 if current_wz > 0.03 else (-1 if current_wz < -0.03 else 0)

        status = ""
        if self.last_wz_sign != 0 and current_sign != 0 and self.last_wz_sign != current_sign:
            self.flip_count += 1
            status = f"[FLIP #{self.flip_count}] wz reversed!"
        elif xy_ok and yaw_ok:
            status = "*** BOTH TOLERANCES MET! GOAL SHOULD COMPLETE ***"
        elif not xy_ok and yaw_ok:
            status = "Yaw reached! But robot is OUTSIDE XY tolerance!"
        elif xy_ok and not yaw_ok:
            status = "XY reached! Turning for yaw..."

        if current_sign != 0:
            self.last_wz_sign = current_sign

        xy_dist_str = f"{xy_dist:.2f}m" if xy_dist is not None else "N/A"
        yaw_err_str = f"{yaw_err_deg:+.1f}°" if yaw_err_deg is not None else "N/A"
        xy_ok_str = "YES" if xy_ok else "NO"
        yaw_ok_str = "YES" if yaw_ok else "NO"
        wz_str = f"{current_wz:+.2f} rad/s"

        print(f"{t_sec % 1000:7.2f}s | {robot_yaw_str:>9} | {goal_yaw_str:>9} | {yaw_err_str:>9} | {xy_dist_str:>8} | {xy_ok_str:^6} | {yaw_ok_str:^7} | {wz_str:>10} | {status}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalDiagnosticNode()
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
