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
        #Use cuz then you can edit parameter within launch file
        #Change parameters while node is running
        #Set arameters during runtime wihtout code changes
        self.declare_parameter('output_dir', '~/boat/data')
        self.declare_parameter('num_frames', 200)
        self.declare_parameter('video_name', 'video_detected_image.mp4')
        
        # Get parameters
        output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value
        )
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
         #Creates four character code for specifying video codec when writing video files

        #Video codec is used to compress video frames, different codec offer various compression ratios and compatibilites
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(video_path, self.fourcc, 20.0, (640, 480))
        
        if not self.out.isOpened():
            self.get_logger().error(f'Failed to open video writer at {video_path}!')
        else:
            self.get_logger().info(f'Recording to {video_path}')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/detected_image',
            self.record_video,
            10
        )
    
    def record_video(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        if self.cur_frame < self.num_frames:
            self.out.write(frame)
            self.cur_frame += 1
            if self.cur_frame % 10 == 0:
                self.get_logger().info(f'Frame {self.cur_frame}/{self.num_frames}')
        else:
            self.get_logger().info('Recording complete!')
            self.out.release()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = RecordVideo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.out is not None:
            node.out.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()