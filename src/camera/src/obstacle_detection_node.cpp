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
        image_subscriber_ = it_->subscribe("/camera/image_raw/throttled", 1, 
            std::bind(&ObstacleDetectionNode::imageCallback, this, std::placeholders::_1));
            
        // Create publisher for detected image
        image_publisher_ = it_->advertise("/camera/detected_image", 1);  // Queue size 1, not 10
        RCLCPP_INFO(this->get_logger(), "Subscriptions and publishers initialized");
    }
private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg){
        RCLCPP_INFO(this->get_logger(), "Running");
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
        cv::Mat grayscale, blur, edges, dilated;
        std::vector<std::vector<cv::Point>> contours;
        if (resize_height_ != 480 && resize_width_ != 640){
            cv::resize(frame, frame, cv::Size(resize_height_, resize_width_));
        } 
        
        cv::cvtColor(frame, grayscale, cv::COLOR_BGR2GRAY);
        //cv::Size(5, 5) is kernel, bigger kernel and higher sigma results in more blur
        //sigma determines how much infleunce distance pixels have
        cv::GaussianBlur(grayscale, blur, cv::Size(5, 5), 1.4);
        cv::Canny(blur, edges, 90, 180);
        
        cv::dilate(edges, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 1);
        cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        drawContours(msg, contours, frame);
        
            
        std::vector<std::vector<cv::Point>> contours_;
        //findContours can only take in single channel
        //find contours is like finding white object from black background 
        //Use RETR_EXTERNAL since I don't want any children
        //Only finds fully closed contours, so fragemented edges are ignored
        std::string file_path = __FILE__;  

        // Find "/src/" and replace everything after with "/data"
        size_t pos = file_path.find("/src/");
        std::string data_path;
        // if (pos != std::string::npos) {
        //     data_path = file_path.substr(0, pos) + "/data";
        //     RCLCPP_INFO(this->get_logger(), "Data path: %s", data_path.c_str());
        //     cv::imwrite(data_path + "/frame.png", frame);
        //     cv::imwrite(data_path + "/canny.png", edges);
        //     cv::imwrite(data_path + "/dilate.png", dilated);
        //     cv::imwrite(data_path + "/blur.png", blur);
        // }
    }
    void testParametersDilate(std::string data_path, const sensor_msgs::msg::Image::ConstSharedPtr& msg, cv::Mat edges, cv_bridge::CvImagePtr cv_ptr){
        for (int y = 1; y < 4; y++){
            for (int i = 3; i < 10; i = i + 2) {
                cv::Mat frame = cv_ptr->image.clone();
                cv::Mat dilated;
                std::vector<std::vector<cv::Point>> contours;
                cv::dilate(edges, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(i, i)), cv::Point(-1, -1), y);
                cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                drawContours(msg, contours, frame);
                cv::imwrite(data_path + "/dilate_" + std::to_string(y) + "_" + std::to_string(i) + ".png", dilated);
                cv::imwrite(data_path + "/contours_" + std::to_string(y) + "_" + std::to_string(i) + ".png", frame);
            }
        }
        
    }

    void testParametersThreshold(std::string data_path, const sensor_msgs::msg::Image::ConstSharedPtr& msg, cv::Mat grayscale, cv_bridge::CvImagePtr cv_ptr){
        for (int i = 20; i < 80; i = i + 10) {
            cv::Mat frame = cv_ptr->image.clone();
            cv::Mat threshold, dilated;
            std::vector<std::vector<cv::Point>> contours;
            //If in grayscale, object are brighter than background, use THRESH_BINARY, if objects are darker than backgroun, use THRESH_BINARY_INV
            cv::threshold(grayscale, threshold, i, 255, cv::THRESH_BINARY_INV);
            cv::dilate(threshold, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 1);
            cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            drawContours(msg, contours, frame);
            cv::imwrite(data_path + "/threshold" + std::to_string(i) + ".png", threshold);
            cv::imwrite(data_path + "/contours" + std::to_string(i) + ".png", frame);
        }
    }
    void testParametersCanny(std::string data_path, const sensor_msgs::msg::Image::ConstSharedPtr& msg, cv::Mat grayscale, cv_bridge::CvImagePtr cv_ptr){
        for (int y = 100; y < 200; y = y + 10){
            for (int x = 30; x < 90; x = x + 10) {
                cv::Mat edges, blur, dilated;
                std::vector<std::vector<cv::Point>> contours;
                cv::Mat frame = cv_ptr->image.clone();
                //Reduces noise to prevent false edges
                cv::GaussianBlur(grayscale, blur, cv::Size(5,5), 1.4);
                //values above y are considred strong edges, definitely kept as white
                //values between x and y are considred and weak and only used if connected to strong edge
                cv::Canny(blur, edges, x, y);
                cv::dilate(edges, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 1);
                cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                drawContours(msg, contours, frame);
                cv::imwrite(data_path + "/canny" + std::to_string(x) + std::to_string(y)+ ".png", edges);
                cv::imwrite(data_path + "/contours_canny" + std::to_string(x) + std::to_string(y) + ".png", frame);
            }
        }
    }
    
    // Helper function to smooth bounding boxes between frames
    cv::Rect smoothBoundingBox(const cv::Rect& prev, const cv::Rect& curr, float alpha) {
        return cv::Rect(
            static_cast<int>(alpha * prev.x + (1 - alpha) * curr.x),
            static_cast<int>(alpha * prev.y + (1 - alpha) * curr.y),
            static_cast<int>(alpha * prev.width + (1 - alpha) * curr.width),
            static_cast<int>(alpha * prev.height + (1 - alpha) * curr.height)
        );
    }

    void drawContours(const sensor_msgs::msg::Image::ConstSharedPtr& msg, 
                     std::vector<std::vector<cv::Point>> contours, 
                     cv::Mat image) {
        
        const double min_area = 200.0;
        const float iou_threshold = 0.3;
        const float smoothing_alpha = 0.7;  // Higher = more weight on previous box (more smoothing)
        
        std::vector<cv::Rect> current_boxes;
        std::vector<cv::Rect> updated_boxes;
        
        // RCLCPP_INFO(this->get_logger(), "Contours size: %zu", contours.size());
        
        // Extract bounding boxes from contours that meet minimum area
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            
            if (area > min_area) {
                cv::Rect bounding_box = cv::boundingRect(contours[i]);
                current_boxes.push_back(bounding_box);
            }
        }
        
        // RCLCPP_INFO(this->get_logger(), "Current boxes: %zu", current_boxes.size());
        
        // If no previous boxes, just use current detections
        if (previous_boxes_.empty()) {
            updated_boxes = current_boxes;
            for (const auto& box : updated_boxes) {
                plotCentroidRectangle(image, box);
            }
        } 
        else {
            // RCLCPP_INFO(this->get_logger(), "Previous boxes: %zu", previous_boxes_.size());
            
            // Track which previous boxes have been matched
            std::vector<bool> previous_matched(previous_boxes_.size(), false);
            
            // Match current boxes with previous boxes
            for (size_t i = 0; i < current_boxes.size(); i++) {
                bool found_match = false;
                int best_match_idx = -1;
                float best_iou = iou_threshold;
                
                // Find best matching previous box
                for (size_t j = 0; j < previous_boxes_.size(); j++) {
                    if (previous_matched[j]) {
                        continue;  // Skip already matched boxes
                    }
                    
                    float iou = calculateIoU(current_boxes[i], previous_boxes_[j]);
                    
                    if (iou > best_iou) {
                        best_iou = iou;
                        best_match_idx = j;
                        found_match = true;
                    }
                }
                
                if (found_match) {
                    // Mark previous box as matched
                    previous_matched[best_match_idx] = true;
                    
                    // Smooth between previous and current position
                    cv::Rect smoothed_box = smoothBoundingBox(
                        previous_boxes_[best_match_idx], 
                        current_boxes[i], 
                        smoothing_alpha
                    );
                    
                    updated_boxes.push_back(smoothed_box);
                    plotCentroidRectangle(image, smoothed_box);
                    
                    // RCLCPP_INFO(this->get_logger(), 
                    //            "Matched box %zu with previous box %d (IoU: %.3f)", 
                    //            i, best_match_idx, best_iou);
                } 
                else {
                    // New detection - no matching previous box
                    updated_boxes.push_back(current_boxes[i]);
                    plotCentroidRectangle(image, current_boxes[i]);
                    
                    
                }
            }
        }
        
        // Update previous boxes for next frame
        previous_boxes_ = updated_boxes;
        // Publish processed image
        sensor_msgs::msg::Image::SharedPtr processed_msg = 
            cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
        image_publisher_.publish(processed_msg);
    }
    void plotCentroidRectangle(cv::Mat image, cv::Rect bounding_box){
        cv::Point centroid(-1, -1);
        cv::rectangle(image, bounding_box, cv::Scalar(255, 255, 0), 2);
        // cv::Moments moments = cv::moments(bounding_box);
        // if (moments.m00 != 0) {
        //     centroid.x = static_cast<int>(moments.m10 / moments.m00);
        //     centroid.y = static_cast<int>(moments.m01 / moments.m00);
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "Invalid moments (m00 = 0)");
        // }
        // cv::circle(image, centroid, 3, cv::Scalar(0, 0, 255), -1);
    }
    float calculateIoU(const cv::Rect& box1, const cv::Rect& box2){
        cv:: Rect intersection = box1&box2;
        //Union considerers both box areas since each box may have a different % interesection with other box
        float inter_area = intersection.area();
        // RCLCPP_INFO(this->get_logger(), "Inter_area: %f", inter_area);

        float union_area = box1.area() + box2.area() - inter_area;
        
        return union_area > 0 ? inter_area / union_area : 0.0;
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
    std::vector<cv::Rect> previous_boxes_;
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