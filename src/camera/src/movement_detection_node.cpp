// src/color_detection_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

class MovementDetectionNode : public rclcpp::Node
{
public:
    MovementDetectionNode() : Node("movement_detection_node")
    {

        // this->declare_parameter<double>("turn_speed", 0.5);
        // this->declare_parameter<int>("min_contour_area", 500);
        
        RCLCPP_INFO(this->get_logger(), "Movement Detection Node Started");

    }
    
    // Initialize subscriptions and publishers after the node is fully constructed
    void initialize()
    {
        // Initialize image transport AFTER the node is managed by shared_ptr
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        
        // Subscribe to camera image topic
        image_subscriber_ = it_->subscribe("/camera/image_raw", 1, 
            std::bind(&MovementDetectionNode::imageCallback, this, std::placeholders::_1));
        
        // Publisher for processed image (for debugging)
        // Advertise publicizes that the node will be publishing messages on the given topic. Creates multicast stream to ensure other nodes receive it
        image_publisher_ = it_->advertise("/camera/processed_movement_image", 1);
        
        // Publisher for navigation commands
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/camera/cmd_vel", 10);
        KNN_subtractor = cv::createBackgroundSubtractorKNN(true);

        // Create MOG2 background subtractor
        //(int history = 100, double varThreshold = 16, bool detectShadows = true
        //
        MOG2_subtractor = cv::createBackgroundSubtractorMOG2(100,16,true);
        bg_subtractor = MOG2_subtractor;
        RCLCPP_INFO(this->get_logger(), "Movement Node Initialized");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;
            cv::Mat foreground_mask, threshold_img, dilated;
            //bg_subtractor is background subtraction, results in foreground_mask
            bg_subtractor->apply(frame, foreground_mask, 0.4);

            // Apply threshold to create a binary image
            cv::threshold(foreground_mask, threshold_img, 80, 255, cv::THRESH_BINARY);

            // Dilate the threshold image to thicken the regions of interest
            cv::dilate(threshold_img, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 1);

            // Find contours in the dilated image
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // Draw bounding boxes for contours that exceed a certain area threshold
            double max_area = 0;
            double min_area = 1000;
            cv::Point centroid(-1, -1);
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
                cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_index]);
                cv::rectangle(frame, bounding_box, cv::Scalar(255, 255, 0), 2);
                
                // Calculate centroid
                cv::Moments moments = cv::moments(contours[largest_contour_index]);
                centroid.x = static_cast<int>(moments.m10 / moments.m00);
                centroid.y = static_cast<int>(moments.m01 / moments.m00);
                cv::circle(frame, centroid, 5, cv::Scalar(0, 0, 255), -1);
                sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
                image_publisher_.publish(processed_msg);
                publishNavigationCommand(centroid.x, frame.cols);

            }
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }



    void publishNavigationCommand(int object_x, int image_width)
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
            // error is positve if on object_x is on right side of camera window 
            //negative if on left side of camera_window
            int error = object_x - center_x;
            
            // Proportional control for turning
            double turn_threshold = image_width * 0.05; // 5% of image width
            
            if (abs(error) < turn_threshold)
            {
                // Object is centered - move forward
                cmd_vel.linear.x = 0.3;
                cmd_vel.angular.z = 90;
                RCLCPP_INFO(this->get_logger(), "Moving forward - object centered");
            }
            else
            {
                // Turn towards object
                cmd_vel.linear.x = 0.1; // Slow forward movement while turning
                // cmd_vel.angular.z = -error * turn_speed / center_x; // Proportional turn
                cmd_vel.angular.z = object_x / 5.33;
                
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
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_publisher_;
        cv::Ptr<cv::BackgroundSubtractor> bg_subtractor;
        cv::Ptr<cv::BackgroundSubtractor> MOG2_subtractor;
        cv::Ptr<cv::BackgroundSubtractor> KNN_subtractor;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovementDetectionNode>();
    
    // Initialize the node after it's managed by shared_ptr
    node->initialize();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
