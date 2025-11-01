#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fstream>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <string>

class ObstacleDetectionNode : public rclcpp::Node
{
public:
    ObstacleDetectionNode() : Node("obstacle_detection_node")
    {
        this->declare_parameter("resize_height", 480);
        this->declare_parameter("resize_width", 640);

        resize_height_ = this->get_parameter("resize_height").as_int();
        resize_width_ = this->get_parameter("resize_width").as_int();
    }
    void initialize()
    {
        //shared_from_this() allows multiple pointers to point to same object (this) so that when one pointer is destroyed, it doesn't destory the object causing all the other pointers to then point to empty memory 
        //Pass shared_from_this into image_transport so it can use the node pointer internally for callbacks and ROS operations 
        //<> specifies type of the things in (), for example std::shared_ptr<int> foo = std::make_shared<int> (10); is the same as std::shared_ptr<int> foo2 (new int(10));
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        image_subscriber_ = it_->subscribe("/camera/image_raw", 1, 
            std::bind(&ObstacleDetectionNode::imageCallback, this, std::placeholders::_1));
            
        // Create publisher for detected image
        image_publisher_ = it_->advertise("/camera/detected_image", 1);  // Queue size 1, not 10
        RCLCPP_INFO(this->get_logger(), "Subscriptions and publishers initialized");
    }
private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg){
        //cv_brdige converts between ROS image and OpenCV messages
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat frame = cv_ptr->image;
        cv::Mat grayscale, dilated, threshold, threshold_tozero_inv;
        if (resize_height_ != 480 && resize_width_ != 640){
            cv::resize(frame, frame, cv::Size(resize_height_, resize_width_));
        } 
        
        cv::cvtColor(frame, grayscale, cv::COLOR_BGR2GRAY);
        //If in grayscale, object are brighter than background, use THRESH_BINARY, if objects are darker than backgroun, use THRESH_BINARY_INV
        cv::threshold(grayscale, threshold, 80, 255, cv::THRESH_BINARY_INV);
        cv::threshold(grayscale, threshold_tozero_inv, 80, 255, cv::THRESH_TOZERO_INV);
        cv::dilate(threshold, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 1);
        
            
        std::vector<std::vector<cv::Point>> contours_;
        //findContours can only take in single channel
        //find contours is like finding white object from black background 
        //Use RETR_EXTERNAL since I don't want any children
        cv::findContours(dilated, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::string file_path = __FILE__;  

        // Find "/src/" and replace everything after with "/data"
        size_t pos = file_path.find("/src/");
        std::string data_path;
        if (pos != std::string::npos) {
            data_path = file_path.substr(0, pos) + "/data";
            RCLCPP_INFO(this->get_logger(), "Data path: %s", data_path.c_str());
            cv::imwrite(data_path + "/frame.png", frame);
            cv::imwrite(data_path + "/grayscale.png", grayscale);
            cv::imwrite(data_path + "/threshold_tozero_inv.png", threshold_tozero_inv);
        }
        
        for (int i = 20; i < 80; i = i + 10) {
            cv::threshold(grayscale, threshold, i, 255, cv::THRESH_BINARY_INV);
            cv::dilate(threshold, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 1);
            cv::findContours(dilated, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            drawContours(msg, contours_, frame);
            cv::imwrite(data_path + "/threshold" + std::to_string(i) + ".png", threshold);
            cv::imwrite(data_path + "/contours" + std::to_string(i) + ".png", frame);
        }
        for (int i = 20; i < 80; i = i + 10) {
            cv::threshold(grayscale, threshold_tozero_inv, i, 255, cv::THRESH_TOZERO_INV);
            cv::dilate(threshold_tozero_inv, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 1);
            cv::findContours(dilated, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            drawContours(msg, contours_, frame);
            cv::imwrite(data_path + "/threshold_tozero_inv" + std::to_string(i) + ".png", threshold);
            cv::imwrite(data_path + "/contours_tozero_inv" + std::to_string(i) + ".png", frame);
        }
    }
    void drawContours(const sensor_msgs::msg::Image::ConstSharedPtr& msg, std::vector<std::vector<cv::Point>> contours, cv::Mat image){
        double min_area = 200;
        cv::Point centroid(-1, -1);
        for (size_t i = 0; i < contours.size(); i++){
            double area = cv::contourArea(contours[i]);
            if (area < min_area){
                continue;
            }
            cv::Rect bounding_box = cv::boundingRect(contours[i]);
            cv::rectangle(image, bounding_box, cv::Scalar(255, 255, 0), 2);
            cv::Moments moments = cv::moments(contours[i]);
            if (moments.m00 != 0) {
                centroid.x = static_cast<int>(moments.m10 / moments.m00);
                centroid.y = static_cast<int>(moments.m01 / moments.m00);
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid moments (m00 = 0)");
            }
            cv::circle(image, centroid, 3, cv::Scalar(0, 0, 255), -1);
        }
        //msg->header contains timestamp when image was captured and frame_id, important metadata that needs to be carried through pipeline, can specify which sensor this message is coming from
        sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
        image_publisher_.publish(processed_msg);

    }
    //<> specificy template parameters, tell container what type of data it holds
    //Ex: std::vector<cv::Point>, a vector that holds OpenCV points, this represents one conotur
    //Since I have multiple contours, that's why I have the nested <std::vector<cv::Point>>
    //vector is in std namespace, not a child class, namespace is like a folder
    //// Multiple contours = vector of vectors
    //example = std::vector<std::vector<cv::Point>> contours = {
        //{Point(10,20), Point(15,25), Point(20,30)},  // Contour 0
        //{Point(50,60), Point(55,65), Point(60,70)},  // Contour 1
        //{Point(100,110), Point(105,115)}             // Contour 2
    //};
    //_ at end of name is a naming convention to indicate member variable
    //makes members variables that are accessible anywhere in class
    std::vector<std::vector<cv::Point>> contours_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher image_publisher_;
    int resize_height_;
    int resize_width_;
    
    
};
//initialize publisher and subscriber
int main(int argc, char** argv){
    //start rclcpp
    //initialize node
    //spian rclcpp with the node
    //end rclcpp
    rclcpp::init(argc, argv);
    //std::make_shared allocates memory for an object and intializes object with supplied arguments, returns pointer
    auto node = std::make_shared<ObstacleDetectionNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}