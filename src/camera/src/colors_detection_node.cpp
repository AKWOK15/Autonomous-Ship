// src/color_detection_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <map>

struct ColorRange {
    int hue_low, hue_high;
    int sat_low, sat_high;
    int val_low, val_high;
    std::string name;
};

class ColorDetectionNode : public rclcpp::Node
{
public:
    ColorDetectionNode() : Node("color_detection_node")
    {
        // Declare parameters for color detection
        declareColorParameters();
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
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/camera/cmd_vel", 10);
        color_publisher_ = this->create_publisher<std_msgs::msg::String>("/camera/detected_color", 10);
        RCLCPP_INFO(this->get_logger(), "Color Detection Node Initialized");
    }

private:
    void declareColorParameters()
    {
        // Method 1: Structured parameter naming (recommended)
        this->declare_parameter<int>("colors.blue.hue_low", 100);
        this->declare_parameter<int>("colors.blue.hue_high", 130);
        this->declare_parameter<int>("colors.blue.sat_low", 50);
        this->declare_parameter<int>("colors.blue.sat_high", 255);
        this->declare_parameter<int>("colors.blue.val_low", 50);
        this->declare_parameter<int>("colors.blue.val_high", 255);
        
        // Red color (note: red wraps around in HSV)
        this->declare_parameter<int>("colors.red.hue_low", 0);
        this->declare_parameter<int>("colors.red.hue_high", 10);
        this->declare_parameter<int>("colors.red.sat_low", 100);
        this->declare_parameter<int>("colors.red.sat_high", 255);
        this->declare_parameter<int>("colors.red.val_low", 100);
        this->declare_parameter<int>("colors.red.val_high", 255);
        
        this->declare_parameter<int>("colors.green.hue_low", 40);
        this->declare_parameter<int>("colors.green.hue_high", 80);
        this->declare_parameter<int>("colors.green.sat_low", 50);
        this->declare_parameter<int>("colors.green.sat_high", 255);
        this->declare_parameter<int>("colors.green.val_low", 50);
        this->declare_parameter<int>("colors.green.val_high", 255);
        
        this->declare_parameter<int>("colors.black.hue_low", 0);
        this->declare_parameter<int>("colors.black.hue_high", 0);
        this->declare_parameter<int>("colors.black.sat_low", 0);
        this->declare_parameter<int>("colors.black.sat_high", 0);
        this->declare_parameter<int>("colors.black.val_low", 0);
        this->declare_parameter<int>("colors.black.val_high", 0);
    }

    void getColorParameters()
    {
        // Method 1: Load structured parameters
        std::vector<std::string> color_names = {"blue", "red", "green", "black"};
        
        for (const auto& color : color_names)
        {
            try {
                ColorRange range;
                range.name = color;
                range.hue_low = this->get_parameter("colors." + color + ".hue_low").as_int();
                range.hue_high = this->get_parameter("colors." + color + ".hue_high").as_int();
                range.sat_low = this->get_parameter("colors." + color + ".sat_low").as_int();
                range.sat_high = this->get_parameter("colors." + color + ".sat_high").as_int();
                range.val_low = this->get_parameter("colors." + color + ".val_low").as_int();
                range.val_high = this->get_parameter("colors." + color + ".val_high").as_int();
                
                color_ranges_[color] = range;
                // RCLCPP_INFO(this->get_logger(), "Loaded color '%s': H(%d-%d), S(%d-%d), V(%d-%d)", 
                //            color.c_str(), range.hue_low, range.hue_high, 
                //            range.sat_low, range.sat_high, range.val_low, range.val_high);
            }
            catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
                RCLCPP_WARN(this->get_logger(), "Color '%s' not configured, skipping", color.c_str());
            }
        }
    }

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
        getColorParameters();
        double turn_speed = this->get_parameter("turn_speed").as_double();
        int min_area = this->get_parameter("min_contour_area").as_int();
        
        // Convert BGR to HSV
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        std::string detected_color = "none";
        cv::Point best_centroid(-1, -1);
        double max_area = 0;
        for (const auto& [color_name, color_range] : color_ranges_) {
                auto [centroid, area] = detectColor(hsv, image, color_range, min_area);
                if (area > max_area) {
                    max_area = area;
                    best_centroid = centroid;
                    detected_color = color_name;
                }
        }
        if (best_centroid.x >= 0) {
            cv::circle(image, best_centroid, 5, cv::Scalar(0, 0, 255), -1);
            publishNavigationCommand(best_centroid.x, image.cols, turn_speed);
            
            cv::putText(image, "Color: " + detected_color, cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::putText(image, "Area: " + std::to_string(static_cast<int>(max_area)), 
                        cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            
            // Publish detected color
            std_msgs::msg::String color_msg;
            color_msg.data = detected_color;
            // RCLCPP_INFO(this->get_logger(), "Color is: %s", color_msg.data.c_str());
            color_publisher_->publish(color_msg);
        }
        else {
            publishNavigationCommand(-1, image.cols, 0.0);
            cv::putText(image, "No Object Found", cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }
    }

    std::pair<cv::Point, double> detectColor(const cv::Mat& hsv, cv::Mat& image, const ColorRange& color_range, int min_area)
    {
        // Apply morphological operations to clean up the mask
        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(color_range.hue_low, color_range.sat_low, color_range.val_low), 
                    cv::Scalar(color_range.hue_high, color_range.sat_high, color_range.val_high), mask);
        //red color wrap around
        if (color_range.name == "red" && color_range.hue_low > color_range.hue_high) {
            cv::Mat mask1, mask2;
            cv::inRange(hsv, cv::Scalar(0, color_range.sat_low, color_range.val_low), 
                        cv::Scalar(color_range.hue_high, color_range.sat_high, color_range.val_high), mask1);
            cv::inRange(hsv, cv::Scalar(color_range.hue_low, color_range.sat_low, color_range.val_low), 
                        cv::Scalar(179, color_range.sat_high, color_range.val_high), mask2);
            cv::bitwise_or(mask1, mask2, mask);
        }
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        
        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        // Chain_APPROX_SIMPLEis countour approximation method
        // RETR External is contour retrival mode
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // Find the largest contour (border of an object)
        double max_area = 0;
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
            cv::drawContours(image, contours, largest_contour_index, cv::Scalar(0, 255, 0), 2);
            
            // Calculate centroid
            cv::Moments moments = cv::moments(contours[largest_contour_index]);
            centroid.x = static_cast<int>(moments.m10 / moments.m00);
            centroid.y = static_cast<int>(moments.m01 / moments.m00);
            
            // Draw centroid
            cv::circle(image, centroid, 5, cv::Scalar(0, 0, 255), -1);
            
        }
        return {centroid, max_area};

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
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_publisher_;
        std::map<std::string, ColorRange> color_ranges_;
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
