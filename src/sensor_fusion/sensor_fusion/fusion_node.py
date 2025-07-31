import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import message_filters
import serial
import message_filters
import threading
import time

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Initialize serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.x = 0
        # Subscribe to both sensors
        self.ultrasonic_sub = message_filters.Subscriber(self, Range, '/ultrasonic/range')
        self.cmd_vel_sub = message_filters.Subscriber(self, Twist, '/camera/cmd_vel')
        self.detected_color_sub = message_filters.Subscriber(self, String, '/camera/detected_color')
        
        # Synchronize messages by timestamp 
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.detected_color_sub, self.cmd_vel_sub, self.ultrasonic_sub],  # Only ROS subscribers here
            queue_size=10,
            slop=0.1,  # 100ms tolerance
            allow_headerless=True
        )
        self.ts.registerCallback(self.sync_callback)

        self.latest_data = None
        self.data_lock = threading.Lock()

        self.timer = self.create_timer(1, self.timer_callback)

    def sync_callback(self, detected_color_msg, vel_msg, ultrasonic_msg):
        with self.data_lock:
                self.latest_data = (detected_color_msg, vel_msg, ultrasonic_msg)

    def timer_callback(self):
        with self.data_lock:
            if self.latest_data is not None:
                detected_color_msg, vel_msg, ultrasonic_msg = self.latest_data
                self.fusion_callback(detected_color_msg, vel_msg, ultrasonic_msg)

    
    def __str__(self):
        return f"Sensor fusion: {self.ts}"

    def fusion_callback(self, detected_color_msg, vel_msg, ultrasonic_msg):
        servo_motor_angle = vel_msg.angular.z
        # Process both sensor data together
        distance = ultrasonic_msg.range
        self.get_logger().info(f'Distance: {distance:.2f} m')
        self.get_logger().info(f'angular.z: {servo_motor_angle:.2f} degrees')
        
        color = detected_color_msg.data
        
        
        # Determine servo angle based on color
        if color == 'black':
            
            self.get_logger().info(f'Servo motor angle: {servo_motor_angle}')
        
            # Send command to Arduino
            self.call_arduino(servo_motor_angle)
        # elif color == "green":
        #     servo_motor_angle = '180'
        # else:
        #     servo_motor_angle = '90'
            
        

    def call_arduino(self, servo_motor_angle):
        try:
            # Send angle as bytes with newline terminator
            if self.x == 0:
                
                self.get_logger().info(f'Made it to first self.x')
                command = f"{180}\n"
                self.x+=1
            elif self.x == 1:
                command = f"{0}\n"
                self.x+=1
            elif self.x == 2:
                command = f"{180}\n"
                self.x+=1
            else:
                command = f"{servo_motor_angle}\n"
            time.sleep(0.2)
            
            # Read any response from Arduino
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8').strip()
                if response:
                    self.get_logger().info(f'Arduino says: {response}')
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