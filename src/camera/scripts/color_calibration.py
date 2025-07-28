import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import threading

class ColorCalibrationNode(Node):
    def __init__(self):
        super().__init__('color_calibration_node')
        self.bridge = CvBridge()
        
        # Throttle image processing to reduce load
        self.last_process_time = 0
        self.process_interval = 0.1  # Process every 100ms
        
        # Track if we need to update the display
        self.needs_update = False
        self.current_image = None
        self.processing_lock = threading.Lock()
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Create trackbars for HSV adjustment
        cv2.namedWindow('Color Calibration', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Color Calibration', 640, 480)
        
        # Use a simple callback that just sets a flag
        cv2.createTrackbar('H Low', 'Color Calibration', 0, 179, self.on_trackbar_change)
        cv2.createTrackbar('H High', 'Color Calibration', 179, 179, self.on_trackbar_change)
        cv2.createTrackbar('S Low', 'Color Calibration', 0, 255, self.on_trackbar_change)
        cv2.createTrackbar('S High', 'Color Calibration', 255, 255, self.on_trackbar_change)
        cv2.createTrackbar('V Low', 'Color Calibration', 0, 255, self.on_trackbar_change)
        cv2.createTrackbar('V High', 'Color Calibration', 255, 255, self.on_trackbar_change)
        
        # Set initial values for red detection
        cv2.setTrackbarPos('H Low', 'Color Calibration', 0)
        cv2.setTrackbarPos('H High', 'Color Calibration', 10)
        cv2.setTrackbarPos('S Low', 'Color Calibration', 50)
        cv2.setTrackbarPos('S High', 'Color Calibration', 255)
        cv2.setTrackbarPos('V Low', 'Color Calibration', 50)
        cv2.setTrackbarPos('V High', 'Color Calibration', 255)
        
        # Start a timer for periodic processing
        self.create_timer(0.05, self.timer_callback)  # 20 FPS max
        
        self.get_logger().info('Color calibration tool started (optimized)')
        self.get_logger().info('Press "q" to quit and print final values')
        self.get_logger().info('Press "s" to save current values')
    
    def on_trackbar_change(self, val):
        """Lightweight callback that just sets update flag"""
        self.needs_update = True
    
    def timer_callback(self):
        """Timer-based processing instead of callback-based"""
        if self.needs_update and self.current_image is not None:
            with self.processing_lock:
                self.process_current_image()
                self.needs_update = False
    
    def get_trackbar_values(self):
        """Get current trackbar values as tuple"""
        return (
            cv2.getTrackbarPos('H Low', 'Color Calibration'),
            cv2.getTrackbarPos('H High', 'Color Calibration'),
            cv2.getTrackbarPos('S Low', 'Color Calibration'),
            cv2.getTrackbarPos('S High', 'Color Calibration'),
            cv2.getTrackbarPos('V Low', 'Color Calibration'),
            cv2.getTrackbarPos('V High', 'Color Calibration')
        )
    
    def image_callback(self, msg):
        try:
            with self.processing_lock:
                # Convert ROS image to OpenCV and store
                self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.needs_update = True
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def process_current_image(self):
        """Process the current image with current trackbar values"""
        if self.current_image is None:
            return
            
        # Get trackbar values
        h_low, h_high, s_low, s_high, v_low, v_high = self.get_trackbar_values()
        
        # Work with a copy to avoid threading issues
        cv_image = self.current_image.copy()
        
        # Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Create mask
        mask = cv2.inRange(hsv, np.array([h_low, s_low, v_low]), 
                          np.array([h_high, s_high, v_high]))
        
        # Apply morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw contours on original image
        result = cv_image.copy()
        
        # Find largest contour and draw centroid
        total_area = 0
        if contours:
            # Only process largest few contours to save time
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:3]
            
            for contour in contours:
                area = cv2.contourArea(contour)
                total_area += area
                
                if area > 100:  # Minimum area threshold
                    # Draw contour
                    cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
                    
                    # Draw centroid
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)
        
        # Add text info
        cv2.putText(result, f"Total Area: {int(total_area)}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(result, f"HSV: {h_low}-{h_high}, {s_low}-{s_high}, {v_low}-{v_high}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Show the result
        cv2.imshow('Color Calibration', result)
        
        # Check for quit or save (non-blocking)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.save_and_quit()
        elif key == ord('s'):
            self.save_current_values()
    
    def save_current_values(self):
        """Save current values without quitting"""
        h_low, h_high, s_low, s_high, v_low, v_high = self.get_trackbar_values()
        self.get_logger().info('Current HSV values:')
        self.get_logger().info(f'hue_low: {h_low}, hue_high: {h_high}')
        self.get_logger().info(f'sat_low: {s_low}, sat_high: {s_high}')
        self.get_logger().info(f'val_low: {v_low}, val_high: {v_high}')
        
        # Also print in parameter format
        self.get_logger().info('Parameter format:')
        self.get_logger().info(f'this->declare_parameter<int>("colors.red.hue_low", {h_low});')
        self.get_logger().info(f'this->declare_parameter<int>("colors.red.hue_high", {h_high});')
        self.get_logger().info(f'this->declare_parameter<int>("colors.red.sat_low", {s_low});')
        self.get_logger().info(f'this->declare_parameter<int>("colors.red.sat_high", {s_high});')
        self.get_logger().info(f'this->declare_parameter<int>("colors.red.val_low", {v_low});')
        self.get_logger().info(f'this->declare_parameter<int>("colors.red.val_high", {v_high});')
    
    def save_and_quit(self):
        """Save values and quit"""
        self.save_current_values()
        self.get_logger().info('Shutting down...')
        rclpy.shutdown()

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