import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class ViewProcessedImageNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/processed_movement_image',
            self.image_callback,
            10
        )

        
        # Create trackbars for HSV adjustment
        cv2.namedWindow('View Processed Image', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('View Processed Image', 640, 480)  # Smaller window
        
     
        self.get_logger().info('Started View Processed Image')
    

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV and store
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow('Processed Image', self.current_image)

            #waits for one millisecond and proceeds if no key is pressed, creates continious video
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    


def main():
    rclpy.init()
    node = ViewProcessedImageNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()