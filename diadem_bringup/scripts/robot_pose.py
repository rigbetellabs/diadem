#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException
import tf2_geometry_msgs
from rclpy.duration import Duration
import time


class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__('robot_pose_publisher')
        
        # Parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('reference_frame', 'map')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('topic_name', 'robot_pose')
        
        # Get parameters
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        
        # Initialize TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher
        self.pose_publisher = self.create_publisher(
            PoseStamped, 
            self.topic_name, 
            10
        )
        
        # Timer for continuous publishing
        self.timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.publish_pose_callback
        )
        
        # Status tracking
        self.last_successful_time = None
        self.consecutive_failures = 0
        
        self.get_logger().info(f'Robot Pose Publisher started')
        self.get_logger().info(f'Publishing {self.base_frame} -> {self.reference_frame} transform')
        self.get_logger().info(f'Topic: {self.topic_name}')
        self.get_logger().info(f'Rate: {self.publish_rate} Hz')

    def publish_pose_callback(self):
        try:
            # Look up the transform
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.reference_frame
            
            # Set position from transform
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            
            # Set orientation from transform
            pose_msg.pose.orientation.x = transform.transform.rotation.x
            pose_msg.pose.orientation.y = transform.transform.rotation.y
            pose_msg.pose.orientation.z = transform.transform.rotation.z
            pose_msg.pose.orientation.w = transform.transform.rotation.w
            
            # Publish the pose
            self.pose_publisher.publish(pose_msg)
            
            # Reset failure counter on success
            if self.consecutive_failures > 0:
                self.get_logger().info(f'Transform lookup successful again after {self.consecutive_failures} failures')
                self.consecutive_failures = 0
            
            self.last_successful_time = self.get_clock().now()
            
        except TransformException as ex:
            self.consecutive_failures += 1
            
            # Log different messages based on failure count to avoid spam
            if self.consecutive_failures == 1:
                self.get_logger().warn(f'Could not transform {self.base_frame} to {self.reference_frame}: {ex}')
            elif self.consecutive_failures % 50 == 0:  # Log every 50 failures
                self.get_logger().warn(f'Still unable to get transform after {self.consecutive_failures} attempts')
            
            # Continue running - don't publish anything this cycle
            pass
            
        except Exception as ex:
            self.get_logger().error(f'Unexpected error in pose publisher: {ex}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        robot_pose_publisher = RobotPosePublisher()
        
        # Keep the node running
        rclpy.spin(robot_pose_publisher)
        
    except KeyboardInterrupt:
        print("\nShutting down Robot Pose Publisher...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()