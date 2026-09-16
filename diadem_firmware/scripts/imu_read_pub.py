#!/usr/bin/env python3

"""
BNO055 UART to ROS2 IMU Publisher Node
Publishes sensor data to sensor_msgs/msg/Imu topic
With improved timing and error handling
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import os
import serial
import time
import math

# ============================================================================
# UART Protocol Constants
# ============================================================================
START_BYTE = 0xAA
WRITE = 0x00
READ = 0x01
RESPONSE_HEADER = 0xEE
WRITE_SUCCESS = 0x01
READ_SUCCESS = 0xBB

# Register Addresses
CHIP_ID = 0x00
BNO055_ID = 0xA0
OPR_MODE = 0x3D
PWR_MODE = 0x3E
UNIT_SEL = 0x3B

# Operation modes
CONFIGMODE = 0x00
IMU = 0x08
NDOF = 0x0C

# Power modes
NORMAL_MODE = 0x00

# Unit selection (m/s², rad/s required for ROS)
METERS_PER_SECOND = 0b00000000
RAD_PER_SECOND = 0b00000010      # Set rad/s for angular velocity
DEG = 0b00000000
CELSIUS = 0b00000000
WINDOWS_ORIENTATION = 0b00000000

# Data registers - read all at once to avoid multiple UART transactions
VECTOR_ALL_DATA = 0x1A           # Start from Euler angles
VECTOR_ALL_DATA_LENGTH = 26      # Euler(6) + Quat(8) + LinAccel(6) + Gyro(6) = 26 bytes

# Individual register addresses (for fallback)
CALIB_STAT = 0x35
VECTOR_QUATERNION = 0x20
VECTOR_GYROSCOPE = 0x14
VECTOR_LINEAR_ACCELERATION = 0x28
VECTOR_EULER = 0x1A

# Conversion scales
QUATERNION_SCALE = 16384.0      # 1 quat = 16384 LSB (2^14)
ANGULAR_RAD_SCALE = 900.0       # 1 rad/s = 900 LSB
LINEAR_SCALE = 100.0            # 1 m/s² = 100 LSB


# ============================================================================
# BNO055 UART Communication Class (Improved)
# ============================================================================

class BNO055_UART:
    """BNO055 UART communication handler with improved timing"""
    
    def __init__(self, port='/dev/imu', baudrate=115200):
        """Initialize UART connection."""
        self.serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.3, # Increased timeout
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        time.sleep(0.5)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()
        
        # Track consecutive failures
        self.consecutive_failures = 0
        self.max_consecutive_failures = 5
    
    def build_write_command(self, address, data):
        """Build UART write command."""
        command = bytearray([START_BYTE, WRITE, address])
        if isinstance(data, int):
            command.append(1)
            command.append(data)
        else:
            command.append(len(data))
            command.extend(data)
        return command
    
    def build_read_command(self, address, length):
        """Build UART read command."""
        return bytearray([START_BYTE, READ, address, length])
    
    def check_response(self, response):
        """Check response validity."""
        if len(response) < 2:
            return False
        return response[0] == READ_SUCCESS or (response[0] == RESPONSE_HEADER and response[1] == WRITE_SUCCESS)
    
    def write_register(self, address, data, retries=3):
        """Write to BNO055 register with retry logic."""
        for attempt in range(retries):
            try:
                command = self.build_write_command(address, data)
                self.serial_port.reset_input_buffer()
                self.serial_port.write(command)
                self.serial_port.flush()
                
                # Wait longer for response (BNO055 can take up to 4ms)
                time.sleep(0.01)
                
                response = self.serial_port.read(2)
                if self.check_response(response):
                    return True
                    
                # Wait before retry
                time.sleep(0.02)
                
            except serial.SerialException:
                time.sleep(0.02)
                
        return False
    
    def read_register(self, address, length, retries=2):
        """Read from BNO055 register with retry logic."""
        for attempt in range(retries):
            try:
                command = self.build_read_command(address, length)
                
                # Clear any stale data
                self.serial_port.reset_input_buffer()
                
                self.serial_port.write(command)
                self.serial_port.flush()
                
                # Wait for BNO055 to prepare response (can take up to 4ms)
                time.sleep(0.010)# 8ms wait
                
                # Read header (2 bytes)
                header = self.serial_port.read(2)
                
                if not self.check_response(header):
                    # Clear buffer and retry
                    self.serial_port.reset_input_buffer()
                    time.sleep(0.01)
                    continue
                
                # Read data bytes
                data = self.serial_port.read(length)
                
                if len(data) == length:
                    self.consecutive_failures = 0
                    return list(data)
                
                # Incomplete data, clear and retry
                self.serial_port.reset_input_buffer()
                time.sleep(0.01)
                
            except serial.SerialException:
                time.sleep(0.02)
        
        self.consecutive_failures += 1
        return None
    
    def read_all_sensor_data(self):
        """
        Read all sensor data in one transaction to minimize UART overhead.
        Reads Euler(6) + Quaternion(8) + Linear Accel(6) + Gyro(6) = 26 bytes
        Starting from register 0x1A
        """
        # Read from 0x1A (Euler) through 0x1F + 0x20-0x27 (Quat) + 0x28-0x2D (Lin Accel)
        # Actually we need to read separately as registers are not continuous
        
        # Better approach: read each vector separately but with proper timing
        quat_data = self.read_register(VECTOR_QUATERNION, 8)
        if quat_data is None:
            return None, None, None
        
        time.sleep(0.003)  # Small delay between reads
        
        gyro_data = self.read_register(VECTOR_GYROSCOPE, 6)
        if gyro_data is None:
            return None, None, None
        
        time.sleep(0.003)
        
        accel_data = self.read_register(VECTOR_LINEAR_ACCELERATION, 6)
        if accel_data is None:
            return None, None, None
        
        return quat_data, gyro_data, accel_data
    
    def get_chip_id(self):
        """Read chip ID."""
        data = self.read_register(CHIP_ID, 1)
        return data[0] if data else None
    
    def set_operation_mode(self, mode):
        """Set operation mode."""
        success = self.write_register(OPR_MODE, mode)
        if success:
            time.sleep(0.025 if mode == CONFIGMODE else 0.01)
        return success
    
    def set_power_mode(self, mode):
        """Set power mode."""
        return self.write_register(PWR_MODE, mode)
    
    def set_units(self):
        """Set units (m/s², rad/s for ROS compatibility)."""
        units = (METERS_PER_SECOND | RAD_PER_SECOND | 
                DEG | CELSIUS | WINDOWS_ORIENTATION)
        return self.write_register(UNIT_SEL, units)
    
    def get_calibration_status(self):
        """Get calibration status."""
        data = self.read_register(CALIB_STAT, 1)
        if data:
            calib = data[0]
            return {
                'sys': (calib >> 6) & 0x03,
                'gyro': (calib >> 4) & 0x03,
                'accel': (calib >> 2) & 0x03,
                'mag': calib & 0x03
            }
        return None
    
    def parse_quaternion(self, data):
        """Parse quaternion from raw data."""
        if data and len(data) == 8:
            w = int.from_bytes(data[0:2], 'little', signed=True) / QUATERNION_SCALE
            x = int.from_bytes(data[2:4], 'little', signed=True) / QUATERNION_SCALE
            y = int.from_bytes(data[4:6], 'little', signed=True) / QUATERNION_SCALE
            z = int.from_bytes(data[6:8], 'little', signed=True) / QUATERNION_SCALE
            return {'w': w, 'x': x, 'y': y, 'z': z}
        return None
    
    def parse_gyroscope(self, data):
        """Parse gyroscope from raw data."""
        if data and len(data) == 6:
            x = int.from_bytes(data[0:2], 'little', signed=True) / ANGULAR_RAD_SCALE
            y = int.from_bytes(data[2:4], 'little', signed=True) / ANGULAR_RAD_SCALE
            z = int.from_bytes(data[4:6], 'little', signed=True) / ANGULAR_RAD_SCALE
            return {'x': x, 'y': y, 'z': z}
        return None
    
    def parse_linear_acceleration(self, data):
        """Parse linear acceleration from raw data."""
        if data and len(data) == 6:
            x = int.from_bytes(data[0:2], 'little', signed=True) / LINEAR_SCALE
            y = int.from_bytes(data[2:4], 'little', signed=True) / LINEAR_SCALE
            z = int.from_bytes(data[4:6], 'little', signed=True) / LINEAR_SCALE
            return {'x': x, 'y': y, 'z': z}
        return None
    
    def close(self):
        """Close serial connection."""
        if self.serial_port.is_open:
            self.serial_port.close()


# ============================================================================
# ROS2 IMU Publisher Node (Improved)
# ============================================================================

class BNO055ImuPublisher(Node):
    """ROS2 node to publish BNO055 IMU data with improved error handling"""
    
    def __init__(self):
        super().__init__('bno055_imu_publisher')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/imu')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('operation_mode', 'NDOF')  # IMU or NDOF
        
        # Get parameters
        self.configured_port = str(self.get_parameter('serial_port').value)
        raw_baudrate = self.get_parameter('baudrate').value
        try:
            self.baudrate = int(raw_baudrate)
        except (ValueError, TypeError):
            self.baudrate = 115200
        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.operation_mode = str(self.get_parameter('operation_mode').value)
        
        # Validate publish rate (BNO055 max is ~100Hz, recommend 50Hz or less)
        if publish_rate > 100:
            self.get_logger().warn(f'Publish rate {publish_rate}Hz is too high, limiting to 100Hz')
            publish_rate = 100.0
        
        # Connection state management
        self.bno = None
        self.connected = False
        self.last_connect_attempt = 0.0
        
        # Create publisher
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        # Statistics tracking
        self.publish_count = 0
        self.error_count = 0
        self.last_status_time = self.get_clock().now()
        
        # Covariance matrices (based on BNO055 datasheet)
        self.orientation_covariance = [
            0.0159, 0.0, 0.0,
            0.0, 0.0159, 0.0,
            0.0, 0.0, 0.0159
        ]
        
        self.angular_velocity_covariance = [
            0.0012, 0.0, 0.0,
            0.0, 0.0012, 0.0,
            0.0, 0.0, 0.0012
        ]
        
        self.linear_acceleration_covariance = [
            0.0017, 0.0, 0.0,
            0.0, 0.0017, 0.0,
            0.0, 0.0, 0.0017
        ]
        
        # Attempt initial connection
        self.connect_sensor()
        
        # Create timer for publishing and auto-reconnecting
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('BNO055 IMU Publisher started')
        self.get_logger().info(f'Port: {self.configured_port}, Baud: {self.baudrate}, Rate: {publish_rate} Hz, Topic: /imu/data')

    def find_imu_port(self):
        """Auto-detect IMU port, prioritizing configured port /dev/imu."""
        if os.path.exists(self.configured_port):
            return self.configured_port
        if os.path.exists('/dev/imu'):
            return '/dev/imu'
        for candidate in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0']:
            if os.path.exists(candidate):
                return candidate
        return None

    def connect_sensor(self):
        """Connect and configure BNO055 sensor automatically without manual intervention."""
        port = self.find_imu_port()
        if not port:
            self.get_logger().warn(
                f'IMU port {self.configured_port} not available yet. Waiting for device...',
                throttle_duration_sec=3.0
            )
            return False

        candidate_bauds = [self.baudrate]
        if 115200 not in candidate_bauds:
            candidate_bauds.append(115200)

        for baud in candidate_bauds:
            try:
                self.get_logger().info(f'Connecting to BNO055 on detected port {port} at {baud} baud...')
                self.bno = BNO055_UART(port=port, baudrate=baud)
                if self.configure_bno055(self.operation_mode):
                    self.connected = True
                    self.baudrate = baud
                    self.get_logger().info(f'BNO055 IMU successfully configured on {port} at {baud} baud')
                    return True
                else:
                    self.close_sensor()
            except Exception as e:
                self.get_logger().warn(f'Failed to connect to BNO055 on {port} at {baud} baud: {e}', throttle_duration_sec=3.0)
                self.close_sensor()

        return False

    def close_sensor(self):
        """Safely close BNO055 serial connection."""
        self.connected = False
        if self.bno:
            try:
                self.bno.close()
            except Exception:
                pass
            self.bno = None
    
    def configure_bno055(self, operation_mode):
        """Configure BNO055 sensor."""
        if not self.bno:
            return False
        # Check chip ID
        chip_id = self.bno.get_chip_id()
        if chip_id is None:
            self.get_logger().warn('No response from BNO055 (chip ID is None)')
            return False
        if chip_id != BNO055_ID:
            self.get_logger().error(f'Invalid chip ID: 0x{chip_id:02X} (expected 0x{BNO055_ID:02X})')
            return False
        self.get_logger().info(f'BNO055 detected! Chip ID: 0x{chip_id:02X}')
        
        # Set config mode
        if not self.bno.set_operation_mode(CONFIGMODE):
            self.get_logger().error('Failed to set CONFIG mode')
            return False
        
        time.sleep(0.05)
        
        # Set power mode
        if not self.bno.set_power_mode(NORMAL_MODE):
            self.get_logger().error('Failed to set NORMAL power mode')
            return False
        
        time.sleep(0.05)
        
        # Set units (m/s², rad/s)
        if not self.bno.set_units():
            self.get_logger().error('Failed to set units')
            return False
        
        time.sleep(0.05)
        
        # Set operation mode
        mode = NDOF if operation_mode == 'NDOF' else IMU
        if not self.bno.set_operation_mode(mode):
            self.get_logger().error(f'Failed to set {operation_mode} mode')
            return False
        
        time.sleep(0.5)  # Wait for mode to stabilize
        self.get_logger().info(f'BNO055 configured in {operation_mode} mode')
        return True
    
    def timer_callback(self):
        """Timer callback to read and publish IMU data or auto-reconnect."""
        if not self.connected:
            now = time.time()
            if now - self.last_connect_attempt >= 1.0:
                self.last_connect_attempt = now
                self.connect_sensor()
            return

        try:
            # Read all sensor data in optimized way
            quat_data, gyro_data, accel_data = self.bno.read_all_sensor_data()
            
            # Check if data is valid
            if quat_data is None or gyro_data is None or accel_data is None:
                self.error_count += 1
                
                # Only log warning occasionally to avoid spam
                if self.error_count % 10 == 1:
                    self.get_logger().warn(
                        f'Failed to read IMU data (errors: {self.error_count}/{self.publish_count + self.error_count})'
                    )
                
                # If too many consecutive failures, try to reset communication or reconnect
                if self.bno.consecutive_failures >= self.bno.max_consecutive_failures:
                    self.get_logger().error('Too many consecutive failures, attempting recovery...')
                    try:
                        self.bno.serial_port.reset_input_buffer()
                        self.bno.serial_port.reset_output_buffer()
                    except Exception:
                        self.close_sensor()
                        return
                    time.sleep(0.1)
                    self.bno.consecutive_failures = 0
                
                return
            
            # Parse data
            quaternion = self.bno.parse_quaternion(quat_data)
            angular_velocity = self.bno.parse_gyroscope(gyro_data)
            linear_acceleration = self.bno.parse_linear_acceleration(accel_data)
            
            if not all([quaternion, angular_velocity, linear_acceleration]):
                self.get_logger().warn('Failed to parse IMU data')
                return
            
            # Create and publish IMU message
            imu_msg = Imu()
            
            # Set header
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Set orientation (quaternion)
            imu_msg.orientation.w = quaternion['w']
            imu_msg.orientation.x = quaternion['x']
            imu_msg.orientation.y = quaternion['y']
            imu_msg.orientation.z = quaternion['z']
            imu_msg.orientation_covariance = self.orientation_covariance
            
            # Set angular velocity (rad/s)
            imu_msg.angular_velocity.x = angular_velocity['x']
            imu_msg.angular_velocity.y = angular_velocity['y']
            imu_msg.angular_velocity.z = angular_velocity['z']
            imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
            
            # Set linear acceleration (m/s²)
            imu_msg.linear_acceleration.x = linear_acceleration['x']
            imu_msg.linear_acceleration.y = linear_acceleration['y']
            imu_msg.linear_acceleration.z = linear_acceleration['z']
            imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance
            
            # Publish message
            self.publisher_.publish(imu_msg)
            self.publish_count += 1
            
            # Periodic status update
            current_time = self.get_clock().now()
            if (current_time - self.last_status_time).nanoseconds > 10e9:  # Every 10 seconds
                total = self.publish_count + self.error_count
                success_rate = (self.publish_count / total * 100) if total > 0 else 0
                self.get_logger().info(
                    f'Stats: {self.publish_count} published, {self.error_count} errors ({success_rate:.1f}% success)'
                )
                self.last_status_time = current_time
            
        except serial.SerialException as e:
            self.get_logger().error(f'Serial exception during IMU read: {e}. Reconnecting...')
            self.close_sensor()
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.get_logger().info(f'Shutting down. Total published: {self.publish_count}, errors: {self.error_count}')
        self.close_sensor()
        super().destroy_node()


# ============================================================================
# Main Function
# ============================================================================

def main(args=None):
    """Main function to run the ROS2 node."""
    rclpy.init(args=args)
    
    try:
        node = BNO055ImuPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
