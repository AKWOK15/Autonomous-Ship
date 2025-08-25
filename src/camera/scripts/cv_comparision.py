import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class CVComparision(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Create windows
        # cv2.namedWindow('MOG2', cv2.WINDOW_NORMAL)
        # cv2.namedWindow('KNN', cv2.WINDOW_NORMAL)
        cv2.namedWindow('MOG2', cv2.WINDOW_NORMAL)
        cv2.namedWindow('KNN', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('MOG2', 640, 480)
        # cv2.resizeWindow('KNN', 640, 480)
        cv2.resizeWindow('MOG2', 640, 480)
        cv2.resizeWindow('KNN', 640, 480)
        
        self.get_logger().info('Started View Processed Image')
        self.MOG2 = cv2.createBackgroundSubtractorMOG2()
        self.KNN = cv2.createBackgroundSubtractorKNN()

    def find_biggest_contour(self, mask):
        """Find the biggest contour in the mask"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the contour with maximum area
            biggest_contour = max(contours, key=cv2.contourArea)
            return biggest_contour, contours
        return None, []

    def draw_rectangle_on_image(self, image, contour, all_contours):
        """Draw rectangles around the biggest contour and all contours"""
        result_image = image.copy()
        
        # Draw rectangles around all contours in gray
        # for cnt in all_contours:
        #     if cv2.contourArea(cnt) > 100:  # Filter small contours
        #         x, y, w, h = cv2.boundingRect(cnt)
        #         cv2.rectangle(result_image, (x, y), (x + w, y + h), (128, 128, 128), 1)
        
        # Draw rectangle around biggest contour in green
        if contour is not None:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
            
            # Add area and dimensions text
            area = cv2.contourArea(contour)
            cv2.putText(result_image, f'Area: {int(area)}', (x, y-30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(result_image, f'Size: {w}x{h}', (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return result_image

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV and store
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Apply background subtraction
            MOG2_mask = self.MOG2.apply(self.current_image)
            KNN_mask = self.KNN.apply(self.current_image)
            
            # Find biggest contours
            mog2_biggest, mog2_all = self.find_biggest_contour(MOG2_mask)
            knn_biggest, knn_all = self.find_biggest_contour(KNN_mask)
            
            # Draw rectangles on original image
            mog2_rectangle_image = self.draw_rectangle_on_image(self.current_image, mog2_biggest, mog2_all)
            knn_rectangle_image = self.draw_rectangle_on_image(self.current_image, knn_biggest, knn_all)
            
            # Display all images
            # cv2.imshow('MOG2', MOG2_mask)
            # cv2.imshow('KNN', KNN_mask)
            cv2.imshow('MOG2', mog2_rectangle_image)
            cv2.imshow('KNN', knn_rectangle_image)
            
            # Wait for one millisecond and proceed if no key is pressed
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main():
    rclpy.init()
    node = CVComparision()
    
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