import cv2
import os
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class RecordVideo(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        
        # Declare parameters
        self.declare_parameter('output_dir', '~/boat/data')
        self.declare_parameter('num_frames', 200)
        self.declare_parameter('endless_recording', True)
        self.declare_parameter('video_name', 'video_detected_image.mp4')
        
        # Get parameters
        output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value
        )
        self.endless_recording = self.get_parameter('endless_recording').value
        self.num_frames = self.get_parameter('num_frames').value
        video_name = self.get_parameter('video_name').value
        video_path = os.path.join(output_dir, video_name)
        os.makedirs(output_dir, exist_ok=True)
        
        # Create output directory
        if os.path.exists(video_path):
            try: 
                os.remove(video_path)
                print("File deleted")
            except OSError as e:
                print(f"Error deleting file: {e}")
        
        self.bridge = CvBridge()
        self.cur_frame = 0
        self.is_recording = True  # Track recording state
        
        # Creates four character code for specifying video codec
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(video_path, self.fourcc, 20.0, (320, 240))
        
        if not self.out.isOpened():
            self.get_logger().error(f'Failed to open video writer at {video_path}!')
        else:
            self.get_logger().info(f'Recording to {video_path}')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/processed_movement_image',
            self.record_video,
            10
        )
    
    def record_video(self, msg):
        if not self.is_recording:
            return
            
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.out.write(frame)
        self.cur_frame += 1
        
        if self.cur_frame % 10 == 0:
            self.get_logger().info(f'Frame {self.cur_frame}/{self.num_frames}')
        
        if self.cur_frame >= self.num_frames and not self.endless_recording:
            self.end_video()
    
    def end_video(self):
        if not self.is_recording:
            return  # Already ended
            
        self.is_recording = False
        self.get_logger().info(f'Recording complete! Saved {self.cur_frame} frames.')
        
        if self.out is not None and self.out.isOpened():
            self.out.release()
            self.get_logger().info('Video file closed successfully')
        
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = RecordVideo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected, shutting down...')
    finally:
        node.end_video()  # This safely handles video closure
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()