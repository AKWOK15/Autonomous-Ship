// src/color_detection_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <string>
#include <map>
#include <filesystem>

class MovementDetectionNode : public rclcpp::Node
{
public:
    MovementDetectionNode() : Node("movement_detection_node")
    {
        // Declare parameters for video recording
        this->declare_parameter<std::string>("output_dir", "/home/aidankwok/boat/data/threshold_tests/");
        this->declare_parameter<int>("max_frames", 400);
        this->declare_parameter<bool>("enable_recording", false);
        
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
        image_publisher_ = it_->advertise("/camera/processed_movement_image", 1);
        
        pro_time_publisher_old_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/camera/processing_time_old", 10);
        pro_time_publisher_new_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/camera/processing_time_new", 10);
        // Publisher for navigation commands
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/camera/cmd_vel", 10);
        KNN_subtractor = cv::createBackgroundSubtractorKNN(true);

        // Create MOG2 background subtractor
        new_bg_subtractor = cv::createBackgroundSubtractorMOG2(50,16,true);
        old_bg_subtractor = cv::createBackgroundSubtractorMOG2(50,16,true);
        RCLCPP_INFO(this->get_logger(), "Movement Node Initialized");
        kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        
        // Pre-allocate all matrices
        resized_image_ = cv::Mat(120, 160, CV_8UC3);
        foreground_mask_ = cv::Mat(120, 160, CV_8UC1);
        threshold_img_ = cv::Mat(120, 160, CV_8UC1);
        dilated_ = cv::Mat(120, 160, CV_8UC1);
        
        // Initialize video recording if enabled
        if (this->get_parameter("enable_recording").as_bool()) {
            initialize_video_writers();
        }
    }
    
    ~MovementDetectionNode()
    {
        release_video_writers();
    }

private:
    // Video recording variables
    std::map<int, cv::VideoWriter> video_writers_;
    std::vector<int> thresholds_ = {60, 80, 100, 120, 140, 160, 180};
    int frame_count_ = 0;
    int max_frames_;
    std::string output_dir_;
    bool recording_enabled_ = false;
    cv::Point2f smoothed_centroid_{-1, -1};
    float smoothing_factor_ = 0.3f;  // Lower = smoother but slower response (0.1-0.5)
    bool first_detection_ = true;
    
    void initialize_video_writers() {
        output_dir_ = this->get_parameter("output_dir").as_string();
        max_frames_ = this->get_parameter("max_frames").as_int();
        
    
        
        for (int threshold : thresholds_) {
            std::string filename = output_dir_ + "threshold_" + 
                                  std::to_string(threshold) + ".mp4";
            
            // Delete existing file if present
            if (std::filesystem::exists(filename)) {
                std::filesystem::remove(filename);
                RCLCPP_INFO(this->get_logger(), "Deleted existing file: %s", filename.c_str());
            }
            
            cv::VideoWriter writer(filename, 
                                  cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                  20.0, 
                                  cv::Size(320, 240));
            
            if (writer.isOpened()) {
                video_writers_[threshold] = std::move(writer);
                RCLCPP_INFO(this->get_logger(), 
                           "Initialized writer for threshold %d at %s", 
                           threshold, filename.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                            "Failed to initialize writer for threshold %d", threshold);
            }
        }
        
        recording_enabled_ = true;
        RCLCPP_INFO(this->get_logger(), "Video recording initialized. Will record %d frames.", max_frames_);
    }
    
    void release_video_writers() {
        for (auto& [threshold, writer] : video_writers_) {
            if (writer.isOpened()) {
                writer.release();
                RCLCPP_INFO(this->get_logger(), 
                           "Released writer for threshold %d", threshold);
            }
        }
        video_writers_.clear();
        recording_enabled_ = false;
    }

    std::pair<cv::Mat, std::vector<std::vector<cv::Point>>> model(
        const cv::Mat frame, 
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        cv::Mat resized_image;
        cv::resize(frame, resized_image, cv::Size(320, 240));
        cv::Mat foreground_mask;
        
        // Apply background subtraction
        new_bg_subtractor->apply(resized_image, foreground_mask, -1);
        
        // If recording is enabled, process and save each threshold
        if (recording_enabled_ && frame_count_ < max_frames_) {
            for (int threshold_value : thresholds_) {
                cv::Mat threshold_img, dilated, threshold_bgr;
                
                // Apply threshold
                cv::threshold(foreground_mask, threshold_img, threshold_value, 255, cv::THRESH_BINARY);
                // After dilation, add closing to fill gaps
                cv::dilate(threshold_img, dilated, 
                        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), 
                        cv::Point(-1, -1), 1);
                cv::morphologyEx(dilated, dilated, cv::MORPH_CLOSE, 
                                cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
                
                cv::cvtColor(dilated, threshold_bgr, cv::COLOR_GRAY2BGR);

                cv::putText(threshold_bgr, 
                           "Threshold: " + std::to_string(threshold_value),
                           cv::Point(10, 30), 
                           cv::FONT_HERSHEY_SIMPLEX, 
                           0.8, 
                           cv::Scalar(0, 255, 255), 
                           2);
                
                cv::putText(threshold_bgr, 
                           "Frame: " + std::to_string(frame_count_ + 1) + "/" + std::to_string(max_frames_),
                           cv::Point(10, 60), 
                           cv::FONT_HERSHEY_SIMPLEX, 
                           0.8, 
                           cv::Scalar(0, 255, 255), 
                           2);
                
                
                
            //     // Dilate
            //     cv::dilate(threshold_img, dilated, 
            //               cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), 
            //               cv::Point(-1, -1), 1);
                
            //     // Find contours
            //     std::vector<std::vector<cv::Point>> contours;
            //     cv::findContours(dilated, contours, cv::RETR_EXTERNAL, 
            //                    cv::CHAIN_APPROX_SIMPLE);
                
            //     // Create visualization image
            //     cv::Mat visualization;
            //     cv::resize(resized_image, visualization, cv::Size(320, 240));
                
            //     // Draw all contours
            //     int largest_contour_index = -1;
            //     double max_area = 0;
            //     double min_area = 50;
            //     for (size_t i = 0; i < contours.size(); i++)
            //     {
            //         double area = cv::contourArea(contours[i]);
            //         if (area > max_area && area > min_area)
            //         {
            //             max_area = area;
            //             largest_contour_index = i;
            //         }
            //     }
        
            // // Process the largest contour if found
            // if (largest_contour_index >= 0){
            //     cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_index]);
            //     cv::rectangle(visualization, bounding_box, cv::Scalar(255, 255, 0), 2);
            // }
                
                // Add text overlay showing threshold and frame count
                // cv::putText(visualization, 
                //            "Threshold: " + std::to_string(threshold_value),
                //            cv::Point(10, 30), 
                //            cv::FONT_HERSHEY_SIMPLEX, 
                //            0.8, 
                //            cv::Scalar(0, 255, 255), 
                //            2);
                
                // cv::putText(visualization, 
                //            "Frame: " + std::to_string(frame_count_ + 1) + "/" + std::to_string(max_frames_),
                //            cv::Point(10, 60), 
                //            cv::FONT_HERSHEY_SIMPLEX, 
                //            0.8, 
                //            cv::Scalar(0, 255, 255), 
                //            2);
                
                // cv::putText(visualization, 
                //            "Contours: " + std::to_string(contours.size()),
                //            cv::Point(10, 90), 
                //            cv::FONT_HERSHEY_SIMPLEX, 
                //            0.8, 
                //            cv::Scalar(0, 255, 255), 
                //            2);
                
                // Write frame to video
                if (video_writers_[threshold_value].isOpened()) {
                    video_writers_[threshold_value].write(threshold_bgr);
                }
            }
            
            frame_count_++;
            
            // Log progress every 20 frames
            if (frame_count_ % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                           "Recording progress: %d/%d frames", 
                           frame_count_, max_frames_);
            }
            
            // Stop recording when done
            if (frame_count_ >= max_frames_) {
                release_video_writers();
                RCLCPP_INFO(this->get_logger(), "Recording complete! Saved %d frames for %zu thresholds.", 
                           max_frames_, thresholds_.size());
            }
        }
        
       
        cv::Mat threshold_img, dilated;
        cv::threshold(foreground_mask, threshold_img, 180, 255, cv::THRESH_BINARY);
        cv::dilate(threshold_img, dilated, 
                  cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), 
                  cv::Point(-1, -1), 1);
        cv::morphologyEx(dilated, dilated, cv::MORPH_CLOSE, 
                                cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated, contours, cv::RETR_EXTERNAL, 
                       cv::CHAIN_APPROX_SIMPLE);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        auto processing_time_msg = std_msgs::msg::Float64MultiArray();
        processing_time_msg.data = {
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                start_time.time_since_epoch()).count()),
            static_cast<double>(duration.count())
        };
        
        publisher_->publish(processing_time_msg);
        return {resized_image, contours};
    }

    std::vector<std::vector<cv::Point>> old_model(const cv::Mat frame, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_){
        auto start_time = std::chrono::high_resolution_clock::now();;
        cv::Mat resized_image;
        cv::resize(frame, resized_image, cv::Size(320, 120));
        cv::Mat foreground_mask, threshold_img, dilated;
        //bg_subtractor is background subtraction, results in foreground_mask
        old_bg_subtractor->apply(resized_image, foreground_mask, -1);
        // Apply threshold to create a binary image, sepearate moving pixels from white pixels 
        cv::threshold(foreground_mask, threshold_img, 80, 255, cv::THRESH_BINARY);

        // Dilate the threshold image to thicken the regions of interest
        cv::dilate(threshold_img, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 1);

        // Find contours in the dilated image
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        auto processing_time_msg = std_msgs::msg::Float64MultiArray();
        processing_time_msg.data = {
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count()),
            static_cast<double>(duration.count())  // microseconds
        };
        publisher_->publish(processing_time_msg);
        return contours;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;
            
            auto [resized_image, contours] = model(frame, pro_time_publisher_new_);

            // Draw bounding boxes for contours that exceed a certain area threshold
            double max_area = 0;
            double min_area = 50;
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
                cv::rectangle(resized_image, bounding_box, cv::Scalar(255, 255, 0), 2);
                
                // Calculate centroid
                cv::Moments moments = cv::moments(contours[largest_contour_index]);
                if (moments.m00 != 0) {
                    cv::Point2f current_centroid;
                    current_centroid.x = static_cast<float>(moments.m10 / moments.m00);
                    current_centroid.y = static_cast<float>(moments.m01 / moments.m00);
                    
                    // Apply exponential moving average
                    if (first_detection_ || smoothed_centroid_.x < 0) {
                        smoothed_centroid_ = current_centroid;
                        first_detection_ = false;
                    } else {
                        smoothed_centroid_.x = smoothing_factor_ * current_centroid.x + 
                                            (1 - smoothing_factor_) * smoothed_centroid_.x;
                        smoothed_centroid_.y = smoothing_factor_ * current_centroid.y + 
                                            (1 - smoothing_factor_) * smoothed_centroid_.y;
                    }
                    
                    // Use smoothed_centroid for servo control
                    centroid.x = static_cast<int>(smoothed_centroid_.x);
                    centroid.y = static_cast<int>(smoothed_centroid_.y);
                    
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid moments (m00 = 0)");
                }
                cv::circle(resized_image, centroid, 3, cv::Scalar(0, 0, 255), -1);
                // toImageMsg() returns sensor_msgs::msg::Image::SharedPtr
                sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(msg->header, "bgr8", resized_image).toImageMsg();
                image_publisher_.publish(processed_msg);
                publishNavigationCommand(centroid.x, resized_image.cols);

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
            int error = object_x - center_x;
            
            // Proportional control for turning
            double turn_threshold = image_width * 0.05; // 5% of image width
            
            if (abs(error) < turn_threshold)
            {
                // Object is centered - move forward
                cmd_vel.linear.x = 0.3;
                cmd_vel.angular.z = 90;
            }
            else
            {
                // Turn towards object
                cmd_vel.linear.x = 0.1;
                cmd_vel.angular.z = object_x / 5.33;
            }
        }
        
        twist_publisher_->publish(cmd_vel);
    }

    cv::Mat resized_image_;
    cv::Mat foreground_mask_;
    cv::Mat threshold_img_;
    cv::Mat dilated_;
    cv::Mat kernel_;
    std::vector<std::vector<cv::Point>> contours_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pro_time_publisher_old_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pro_time_publisher_new_;
    cv::Ptr<cv::BackgroundSubtractor> new_bg_subtractor;
    cv::Ptr<cv::BackgroundSubtractor> old_bg_subtractor;
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