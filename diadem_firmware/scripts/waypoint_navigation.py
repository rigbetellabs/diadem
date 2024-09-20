#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from mavros_msgs.msg import WaypointList, Waypoint, CommandCode, WaypointReached
from mavros_msgs.srv import WaypointPush, WaypointClear, CommandBool, SetMode, StreamRate, CommandTOL
from sensor_msgs.msg import NavSatFix
global_position = NavSatFix()

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('self')

        self.set_stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.waypoint_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.global_position_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.global_position_callback,
            qos_profile
        )
        self.waypoint_reached_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_reached_callback,
            qos_profile
        )
        self.mission_complete = False
        self.waypoint_count = 0
        
        self.get_logger().info("WaypointNavigator Initiated")
        self.set_stream_rate_client.wait_for_service()
        stream_rate_req = StreamRate.Request()
        stream_rate_req.stream_id = 0
        stream_rate_req.message_rate = 10
        stream_rate_req.on_off = True

        future = self.set_stream_rate_client.call_async(stream_rate_req)

        # Wait for the result
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Stream rate set successfully")
            self.run_test()
        else:
            self.get_logger().error("Failed to set stream rate")
        
        
    def waypoint_reached_callback(self, msg):
        if msg.wp_seq == self.waypoint_count - 1:
            self.get_logger().info("Mission completed")
            self.mission_complete = True
            
    def global_position_callback(self, data):
        self.get_logger().info(f"Global position: ({data.latitude}, {data.longitude}, {data.altitude})")
        if data.latitude == 0 and data.longitude == 0 and data.altitude == 0:
            self.get_logger().warn("No GPS fix")
            return False
        self.global_position = data
        return True
        
    def takeoff(self):
        try:
            takeoff_req = CommandTOL.Request()
            takeoff_req.altitude = 10.0
            self.takeoff_client.call_async(takeoff_req)
            self.get_logger().info("Vehicle taking off")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")        
        
    def setmode(self,mode):
        self.set_mode_client.wait_for_service()
        try:
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = mode
            self.set_mode_client.call_async(set_mode_req)
            self.get_logger().info(f"Mode set: {mode}")
            rclpy.spin_until_future_complete(self, self.set_mode_client.call_async(set_mode_req))
        except Exception as e:
            self.get_logger().error(f"Set Mode failed: {e}")
        
    def arm_vehicle(self, arm):
        self.arm_client.wait_for_service()
        try:
            arm_req = CommandBool.Request()
            arm_req.value = arm
            future = self.arm_client.call_async(arm_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                self.get_logger().info("Vehicle %s" % ("armed" if arm else "disarmed"))
            else:
                self.get_logger().warn("Failed to %s vehicle" % ("arm" if arm else "disarm"))
        except Exception as e:
            self.get_logger().error("Service call failed: %s" % str(e))
    
    def goToLocation(self,lat,long):
        self.get_logger().info("Disarming the vehicle to send goals")
        self.arm_vehicle(False)
        time.sleep(3)
        self.setmode('GUIDED')
        self.get_logger().info("AUTO mode activated")
        # Clear existing waypoints
        self.waypoint_clear_client.wait_for_service()
        try:
            clear_req = WaypointClear.Request()
            future_clear = self.waypoint_clear_client.call_async(clear_req)
            rclpy.spin_until_future_complete(self, future_clear)
            if future_clear.result().success:
                self.get_logger().info("Waypoint plan cleared")
            else:
                self.get_logger().warn("Failed to clear waypoint plan")
        except Exception as e:
            self.get_logger().error("Service call failed: %s" % str(e))

        waypoint_list = []

        waypoint1 = Waypoint()
        waypoint1.frame = Waypoint.FRAME_GLOBAL
        waypoint1.command = CommandCode.NAV_TAKEOFF
        waypoint1.is_current = False
        waypoint1.autocontinue = True
        waypoint1.param1 = 0.0
        waypoint1.param2 = 0.0
        waypoint1.param3 = 0.0
        waypoint1.param4 = 0.0
        waypoint1.x_lat = global_position.latitude
        waypoint1.y_long = global_position.longitude
        waypoint1.z_alt = global_position.altitude
        
        waypoint2 = Waypoint()
        waypoint2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint2.command = CommandCode.NAV_WAYPOINT
        waypoint2.is_current = False
        waypoint2.autocontinue = True
        waypoint2.param1 = 0.0
        waypoint2.param2 = 0.0
        waypoint2.param3 = 0.0
        waypoint2.param4 = 0.0
        waypoint2.x_lat = lat
        waypoint2.y_long = long
        waypoint2.z_alt = global_position.altitude
        
        waypoint_list = [waypoint1, waypoint2]
        self.waypoint_count = len(waypoint_list)
        
        # Push waypoints
        self.waypoint_push_client.wait_for_service()
        try:
            push_req = WaypointPush.Request()
            push_req.start_index = 0
            push_req.waypoints = waypoint_list
            future_push = self.waypoint_push_client.call_async(push_req)
            rclpy.spin_until_future_complete(self, future_push)
            if future_push.result().success:
                self.get_logger().info("Waypoint plan pushed successfully, arming vehicle")
                self.arm_vehicle(True)
                self.setmode('AUTO')
                self.takeoff()
                while not self.mission_complete:
                    self.get_logger().info("Mission in progress..")
                    rclpy.spin_once(self)
                    time.sleep(1)
                self.get_logger().info("Mission Complete!")
                self.mission_complete=False
            else:
                self.get_logger().warn("Failed to push waypoint plan")
                
        except Exception as e:
            self.get_logger().error("Service call failed: %s" % str(e))
            
    def run_test(self):
        # Test sending waypoint plan
        self.get_logger().info("Sending waypoints")
        self.goToLocation(18.6173791,73.9099072)
        time.sleep(3)
        self.setmode('RTL')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
