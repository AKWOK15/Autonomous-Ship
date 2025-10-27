import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
import time
import os
class ProcessingTime(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.declare_parameter('output_dir', '~/boat/data')
        self.declare_parameter('plot_name', 'processing_time.png')
        
        # Get parameters
        output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value
        )
        plot_name = self.get_parameter('plot_name').value
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        self.plot_path = os.path.join(output_dir, plot_name)
        self.bridge = CvBridge()
        self.subscription_old = self.create_subscription(
            Float64MultiArray,
            '/camera/processing_time_old',
            lambda msg: self.aggregate_processing_time(msg, True),
            10
        )

        self.subscription_new = self.create_subscription(
            Float64MultiArray,
            '/camera/processing_time_new',
            lambda msg: self.aggregate_processing_time(msg, False),
            10
        )
        self.old_time = []
        self.old_processing_time = []
        self.new_time = []
        self.new_processing_time= []
        

    def aggregate_processing_time(self, msg, is_old):
        if len(self.old_time) > 100:
            plt.plot(self.old_time, self.old_processing_time, color="blue", label="old")
            plt.plot(self.new_time, self.new_processing_time, color="red", label="new")
            plt.title("Processing Time")
            plt.show()
            plt.savefig(self.plot_path)
            self.get_logger().info('Done')
            rclpy.shutdown()
        if len(self.old_time)%10 == 0:
            print(len(self.old_time))
        if is_old:
            self.old_time.append(msg.data[0])
            self.old_processing_time.append(msg.data[1])
        else:
            self.new_time.append(msg.data[0])
            self.new_processing_time.append(msg.data[1])

        

def main():
    #Initialie ROS communicaiton for given context 
    rclpy.init()
    #Sets up node with publishers and subscribers
    node = ProcessingTime()
    print('Starting')
    
    try:
        #Process callabacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #Prevents memory links
        node.destroy_node()
        #Without shutdown, context not cleaned up
        #Can cause issues if you run another ROS2 program in same process 
        rclpy.shutdown()

if __name__ == '__main__':
    main()