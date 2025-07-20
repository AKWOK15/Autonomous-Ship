import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image
from geometry_msgs.msg import Twist
import message_filters 

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
       
        # Subscribe to both sensors
        self.ultrasonic_sub = message_filters.Subscriber(self, Range, '/ultrasonic/range')
        self.color_detection_sub = message_filters.Subscriber(self, Twist, '/camera/cmd_vel')
        # self.camera_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
       
        # Synchronize messages by timestamp
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_detection_sub, self.ultrasonic_sub],
            allow_headerless=True,
            queue_size=10,
            slop=3000
        )
        self.ts.registerCallback(self.fusion_callback)
    
    def __str__(self):
        return f"Sensor fusion: {self.ts}"
   
    # def get_ultrasonic
    def fusion_callback(self, color_detection_msg, ultrasonic_msg):
        # Process both sensor data together
        distance = ultrasonic_msg.range
        self.get_logger().info(f'Distance: {distance:.2f}m')
        camera_linear = color_detection_msg.linear.x
        self.get_logger().info(f'Camera linear: {camera_linear}')
        camera_angular = color_detection_msg.angular.x
        self.get_logger().info(f'Camera linear: {camera_angular}')
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