#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from mavros_msgs.msg import RCOut, RCIn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

MIN_RC_VALUE = 1000
MAX_RC_VALUE = 2000
RC_SETPOINT = 1500

# Define the min and max velocities (in meters per second)
MIN_VELOCITY = -1.0
MAX_VELOCITY = 1.0

wheel_separation = 0.83  # in meters


class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')

        self.control_enabled = False

        # Cache the last computed Twist so we can republish it at 100 Hz
        # even when no new RC data has arrived.
        self._last_cmd_vel = Twist()

        # Create a publisher for cmd_vel messages
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 100)

        # Subscribe to the RC output topic (arrives at ~1 Hz from Pixhawk)
        self.rc_subscriber = self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self.rc_callback,
            10)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10  # You can increase this depth based on your requirement
        )

        # Subscribe to the pixhawk/control topic
        self.control_subscriber = self.create_subscription(
            Bool,
            '/pixhawk/control',
            self.control_callback,
            qos_profile)

        # 100 Hz timer — publishes the last known cmd_vel continuously
        self._publish_timer = self.create_timer(1.0 / 100.0, self._publish_cmd_vel)

        self.get_logger().info("Pixhawk-to-CMD Initiated (publishing cmd_vel at 100 Hz)")

    # ------------------------------------------------------------------
    # RC callback: only *updates* the cached Twist; actual publishing is
    # done by the 100 Hz timer below.
    # ------------------------------------------------------------------
    def rc_callback(self, data):
#        if not self.control_enabled:
#            return
        print("somthing")
        left_channel = data.channels[0]
        right_channel = data.channels[2]

        # Clamp RC values to be within MIN_RC_VALUE and MAX_RC_VALUE
        left_channel = min(MAX_RC_VALUE, max(MIN_RC_VALUE, left_channel))
        right_channel = min(MAX_RC_VALUE, max(MIN_RC_VALUE, right_channel))

        # Scale RC values to the velocity range
        left_velocity = MIN_VELOCITY + (left_channel - MIN_RC_VALUE) * (MAX_VELOCITY - MIN_VELOCITY) / (MAX_RC_VALUE - MIN_RC_VALUE)
        right_velocity = MIN_VELOCITY + (right_channel - MIN_RC_VALUE) * (MAX_VELOCITY - MIN_VELOCITY) / (MAX_RC_VALUE - MIN_RC_VALUE)

        linear_velocity = (left_velocity + right_velocity) / 2
        angular_vel = -(2 * (right_velocity - linear_velocity)) / wheel_separation

        # Update the cached Twist (timer will publish it at 100 Hz)
        self._last_cmd_vel.linear.x = linear_velocity
        self._last_cmd_vel.angular.z = angular_vel

    # ------------------------------------------------------------------
    # Timer callback: fires at 100 Hz and publishes the last known Twist
    # ------------------------------------------------------------------
    def _publish_cmd_vel(self):
        self.cmd_vel_publisher.publish(self._last_cmd_vel)

    def control_callback(self, data):
        self.control_enabled = data.data
        if self.control_enabled:
            self.get_logger().warn("PIXHAWK CONTROL ON")
        else:
            self.get_logger().warn("PIXHAWK CONTROL OFF")

def main(args=None):
    rclpy.init(args=args)
    rover_controller = RoverController()
    rclpy.spin(rover_controller)
    rover_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
