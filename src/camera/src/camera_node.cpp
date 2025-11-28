// Not being used by launch file
// src/camera_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        // Declare parameters
        this->declare_parameter<int>("camera_id", 0);
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);

        this->declare_parameter<std::string>("frame_id", "camera_frame");
        
        RCLCPP_INFO(this->get_logger(), "Camera Node Created");
    }
    
    bool initialize()
    {
        // Get parameters
        int camera_id = this->get_parameter("camera_id").as_int();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // Initialize camera
        cap_.open(camera_id);
        
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera with ID: %d", camera_id);
            return false;
        }
        
        // Set camera properties
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        
        RCLCPP_INFO(this->get_logger(), "Camera opened successfully");
        RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d", width, height);
        
        // Initialize image transport - NOW it's safe to use shared_from_this()
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        
        // Create publisher
        image_publisher_ = it_->advertise("/camera/image_raw", 1);
        
        // Create timer to capture and publish images
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // ~30 FPS
            std::bind(&CameraNode::captureAndPublish, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Camera node initialized and publishing to /camera/image_raw");
        return true;
    }
    
    ~CameraNode()
    {
        if (cap_.isOpened())
        {
            cap_.release();
        }
    }

private:
    void captureAndPublish()
    {
        cv::Mat frame;
        
        if (!cap_.read(frame))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
            return;
        }
        
        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame");
            return;
        }
        
        // Convert OpenCV image to ROS message
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = frame_id_;
        
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            header, "bgr8", frame
        ).toImageMsg();
        
        // Publish the image
        image_publisher_.publish(msg);
        
        // Optional: Log occasionally to show it's working
        static int frame_count = 0;
        if (++frame_count % 150 == 0)  // Every 5 seconds at 30fps
        {
            RCLCPP_INFO(this->get_logger(), "Published %d frames", frame_count);
        }
    }
    
    cv::VideoCapture cap_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string frame_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    
    // Initialize the node after it's managed by shared_ptr
    if (!node->initialize())
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to initialize camera node");
        return 1;
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
