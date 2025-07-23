import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import message_filters
import serial

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Initialize serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        
        # Subscribe to both sensors
        self.ultrasonic_sub = message_filters.Subscriber(self, Range, '/ultrasonic/range')
        self.cmd_el_sub = message_filters.Subscriber(self, Twist, '/camera/cmd_vel')
        self.detected_color_sub = message_filters.Subscriber(self, String, '/camera/detected_color')
        
        # Synchronize messages by timestamp (remove self.ser from synchronizer)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.detected_color_sub, self.ultrasonic_sub],  # Only ROS subscribers here
            queue_size=10,
            slop=0.1,  # 100ms tolerance
            allow_headerless=True
        )
        self.ts.registerCallback(self.fusion_callback)

    def __str__(self):
        return f"Sensor fusion: {self.ts}"

    def fusion_callback(self, detected_color_msg, ultrasonic_msg):
        # Process both sensor data together
        distance = ultrasonic_msg.range
        self.get_logger().info(f'Distance: {distance:.2f}m')
        
        color = detected_color_msg.data
        self.get_logger().info(f'Color of detected object: {color}')
        
        # Determine servo angle based on color
        if color == 'black':
            servo_motor_angle = '0'
        elif color == "green":
            servo_motor_angle = '180'
        else:
            servo_motor_angle = '90'
            
        self.get_logger().info(f'Servo motor angle: {servo_motor_angle}')
        
        # Send command to Arduino
        self.call_arduino(servo_motor_angle)

    def call_arduino(self, servo_motor_angle):
        try:
            # Send angle as bytes with newline terminator
            command = f"{servo_motor_angle}\n"
            self.ser.write(command.encode())
            self.get_logger().info(f'Sent to Arduino: {servo_motor_angle}')
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