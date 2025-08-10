import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import message_filters
import serial
import message_filters
import time
import threading

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Initialize serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        # Subscribe to both sensors
        # self.ultrasonic_sub = message_filters.Subscriber(self, Range, '/ultrasonic/range')
        self.cmd_vel_sub = message_filters.Subscriber(self, Twist, '/camera/cmd_vel')
        # self.detected_color_sub = message_filters.Subscriber(self, String, '/camera/detected_color')
        
        # Synchronize messages by timestamp 
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.cmd_vel_sub],  # Only ROS subscribers here
            queue_size=10,
            slop=0.1,  # 100ms tolerance
            allow_headerless=True
        )
        self.ts.registerCallback(self.sync_callback)

        self.latest_data = None
        self.data_lock = threading.Lock()

        self.timer = self.create_timer(2.0, self.timer_callback)

    
    def __str__(self):
        return f"Sensor fusion: {self.ts}"


    

    def sync_callback(self, vel_msg):
        with self.data_lock:
                self.latest_data = vel_msg

    def timer_callback(self):
        with self.data_lock:
            if self.latest_data is not None:
                self.fusion_callback(self.latest_data)
                
    def fusion_callback(self, vel_msg):
        servo_motor_angle = vel_msg.angular.z
        # Process both sensor data together
        # distance = ultrasonic_msg.range
        # self.get_logger().info(f'Distance: {distance:.2f} m')
        self.call_arduino(servo_motor_angle)
            
        

    def call_arduino(self, servo_motor_angle):
        try:
            # Clear any stale data in the input buffer before sending new command
            if self.ser.in_waiting > 0:
                stale_data = self.ser.read(self.ser.in_waiting)
                self.get_logger().info(f'Cleared {len(stale_data)} stale bytes from buffer')
            
            # Send angle as bytes with newline terminator
            self.get_logger().info(f'Sending angular.z: {servo_motor_angle:.2f} degrees')
            command = f"{servo_motor_angle}\n"
            self.ser.write(command.encode())
            self.ser.flush()  # Ensure data is sent immediately
            
            # Wait longer for Arduino to fully process and respond
            time.sleep(0.5)  # Give Arduino 500ms to process
            
            # Read response from Arduino
            if self.ser.in_waiting > 0:
                self.get_logger().info(f'self.ser.in_waiting: {self.ser.in_waiting} bytes')
                
                # Read all available data and split into lines
                all_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                
                # Split by newlines and process each complete message
                lines = all_data.split('\n')
                for line in lines:
                    line = line.strip()
                    if line:  # Only log non-empty lines
                        self.get_logger().info(f'Arduino says: {line}')
            else:
                self.get_logger().warn(f'No response received for angle {servo_motor_angle}')
        except Exception as e:
            self.get_logger().error(f'Failed to write to Arduino: {e}')

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()
    print(fusion_node)
    print('made it to fusion_node')
    
    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.ser.close()  # Close serial connection
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()