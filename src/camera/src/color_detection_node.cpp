// src/color_detection_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

class ColorDetectionNode : public rclcpp::Node
{
public:
    ColorDetectionNode() : Node("color_detection_node")
    {
        // Declare parameters for color detection
        this->declare_parameter<int>("hue_low", 100);      // Blue color range
        this->declare_parameter<int>("hue_high", 130);
        this->declare_parameter<int>("sat_low", 50);
        this->declare_parameter<int>("sat_high", 255);
        this->declare_parameter<int>("val_low", 50);
        this->declare_parameter<int>("val_high", 255);
        this->declare_parameter<double>("turn_speed", 0.5);
        this->declare_parameter<int>("min_contour_area", 500);
        
        RCLCPP_INFO(this->get_logger(), "Color Detection Node Started");
    }
    
    // Initialize subscriptions and publishers after the node is fully constructed
    void initialize()
    {
        // Initialize image transport AFTER the node is managed by shared_ptr
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        
        // Subscribe to camera image topic
        image_subscriber_ = it_->subscribe("/camera/image_raw", 1, 
            std::bind(&ColorDetectionNode::imageCallback, this, std::placeholders::_1));
        
        // Publisher for processed image (for debugging)
        image_publisher_ = it_->advertise("/camera/processed_image", 1);
        
        // Publisher for navigation commands
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Color Detection Node Initialized");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            
            // Process the image for color detection
            processImage(image);
            
            // Publish processed image for debugging
            sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
            image_publisher_.publish(processed_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void processImage(cv::Mat& image)
    {
        // Get parameters
        int hue_low = this->get_parameter("hue_low").as_int();
        int hue_high = this->get_parameter("hue_high").as_int();
        int sat_low = this->get_parameter("sat_low").as_int();
        int sat_high = this->get_parameter("sat_high").as_int();
        int val_low = this->get_parameter("val_low").as_int();
        int val_high = this->get_parameter("val_high").as_int();
        double turn_speed = this->get_parameter("turn_speed").as_double();
        int min_area = this->get_parameter("min_contour_area").as_int();
        
        // Convert BGR to HSV
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        
        // Create mask for specified color range
        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(hue_low, sat_low, val_low), 
                   cv::Scalar(hue_high, sat_high, val_high), mask);
        
        // Apply morphological operations to clean up the mask
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        
        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // Find the largest contour
        double max_area = 0;
        int largest_contour_index = -1;
        
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if (area > max_area && area > min_area)
            {
                max_area = area;
                largest_contour_index = i;
            }
        }
        
        // Process the largest contour if found
        if (largest_contour_index >= 0)
        {
            // Draw contour on original image
            cv::drawContours(image, contours, largest_contour_index, cv::Scalar(0, 255, 0), 2);
            
            // Calculate centroid
            cv::Moments moments = cv::moments(contours[largest_contour_index]);
            int cx = static_cast<int>(moments.m10 / moments.m00);
            int cy = static_cast<int>(moments.m01 / moments.m00);
            
            // Draw centroid
            cv::circle(image, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);
            
            // Calculate navigation command
            publishNavigationCommand(cx, image.cols, turn_speed);
            
            // Add text information
            cv::putText(image, "Object Found", cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::putText(image, "Area: " + std::to_string(static_cast<int>(max_area)), 
                       cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
        else
        {
            // No object found - publish stop command
            publishNavigationCommand(-1, image.cols, 0.0);
            cv::putText(image, "No Object Found", cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }
    }
    
    void publishNavigationCommand(int object_x, int image_width, double turn_speed)
    {
        geometry_msgs::msg::Twist cmd_vel;
        
        if (object_x < 0)
        {
            // No object detected - stop
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
        }
        else
        {
            // Object detected - calculate turn direction
            int center_x = image_width / 2;
            int error = object_x - center_x;
            
            // Proportional control for turning
            double turn_threshold = image_width * 0.1; // 10% of image width
            
            if (abs(error) < turn_threshold)
            {
                // Object is centered - move forward
                cmd_vel.linear.x = 0.3;
                cmd_vel.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Moving forward - object centered");
            }
            else
            {
                // Turn towards object
                cmd_vel.linear.x = 0.1; // Slow forward movement while turning
                cmd_vel.angular.z = -error * turn_speed / center_x; // Proportional turn
                
                if (error > 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Turning right - object at x=%d", object_x);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Turning left - object at x=%d", object_x);
                }
            }
        }
        
        twist_publisher_->publish(cmd_vel);
    }
    
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorDetectionNode>();
    
    // Initialize the node after it's managed by shared_ptr
    node->initialize();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
