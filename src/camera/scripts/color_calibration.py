#!/usr/bin/env python3
# scripts/color_calibration.py
# Run this script to calibrate color detection parameters

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorCalibrationNode(Node):
    def __init__(self):
        super().__init__('color_calibration_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Create trackbars for HSV adjustment
        cv2.namedWindow('Color Calibration')
        cv2.createTrackbar('H Low', 'Color Calibration', 0, 179, self.nothing)
        cv2.createTrackbar('H High', 'Color Calibration', 179, 179, self.nothing)
        cv2.createTrackbar('S Low', 'Color Calibration', 0, 255, self.nothing)
        cv2.createTrackbar('S High', 'Color Calibration', 255, 255, self.nothing)
        cv2.createTrackbar('V Low', 'Color Calibration', 0, 255, self.nothing)
        cv2.createTrackbar('V High', 'Color Calibration', 255, 255, self.nothing)
        
        # Set initial values for blue detection
        cv2.setTrackbarPos('H Low', 'Color Calibration', 100)
        cv2.setTrackbarPos('H High', 'Color Calibration', 130)
        cv2.setTrackbarPos('S Low', 'Color Calibration', 50)
        cv2.setTrackbarPos('V Low', 'Color Calibration', 50)
        
        self.get_logger().info('Color calibration tool started')
        self.get_logger().info('Press "q" to quit and print final values')
    
    def nothing(self, x):
        pass
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Get trackbar values
            h_low = cv2.getTrackbarPos('H Low', 'Color Calibration')
            h_high = cv2.getTrackbarPos('H High', 'Color Calibration')
            s_low = cv2.getTrackbarPos('S Low', 'Color Calibration')
            s_high = cv2.getTrackbarPos('S High', 'Color Calibration')
            v_low = cv2.getTrackbarPos('V Low', 'Color Calibration')
            v_high = cv2.getTrackbarPos('V High', 'Color Calibration')
            
            # Convert to HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create mask
            mask = cv2.inRange(hsv, np.array([h_low, s_low, v_low]), 
                              np.array([h_high, s_high, v_high]))
            
            # Apply morphological operations
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw contours on original image
            result = cv_image.copy()
            cv2.drawContours(result, contours, -1, (0, 255, 0), 2)
            
            # Find largest contour and draw centroid
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 100:  # Minimum area threshold
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(result, f"Area: {int(area)}", (10, 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show images
            cv2.imshow('Original', cv_image)
            cv2.imshow('Mask', mask)
            cv2.imshow('Result', result)
            
            # Check for quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Final HSV values:')
                self.get_logger().info(f'hue_low: {h_low}')
                self.get_logger().info(f'hue_high: {h_high}')
                self.get_logger().info(f'sat_low: {s_low}')
                self.get_logger().info(f'sat_high: {s_high}')
                self.get_logger().info(f'val_low: {v_low}')
                self.get_logger().info(f'val_high: {v_high}')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main():
    rclpy.init()
    node = ColorCalibrationNode()
    
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
