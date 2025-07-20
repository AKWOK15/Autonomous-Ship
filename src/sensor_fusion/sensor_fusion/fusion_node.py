import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import message_filters 

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
       
        # Subscribe to both sensors
        self.ultrasonic_sub = message_filters.Subscriber(self, Range, '/ultrasonic/range')
        self.cmd_el_sub = message_filters.Subscriber(self, Twist, '/camera/cmd_vel')
        self.detected_color_sub = message_filters.Subscriber(self, String, '/camera/detected_color')
        # self.camera_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
       
        # Synchronize messages by timestamp
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.detected_color_sub, self.ultrasonic_sub],
            allow_headerless=True,
            queue_size=10,
            slop=3000
        )
        self.ts.registerCallback(self.fusion_callback)
    
    def __str__(self):
        return f"Sensor fusion: {self.ts}"
   
    # def get_ultrasonic
    def fusion_callback(self, detected_color_msg, ultrasonic_msg):
        # Process both sensor data together
        distance = ultrasonic_msg.range
        self.get_logger().info(f'Distance: {distance:.2f}m')
        color = detected_color_msg.data
        self.get_logger().info(f'Color of detected object: {color}')
        # Process camera image...
        # Fuse the data and publish result

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()
    print(fusion_node)
    print('made it to fusion_node')
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()