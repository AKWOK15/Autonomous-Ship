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
#include <ament_index_cpp/get_package_share_directory.hpp>

class ObstacleDetectionNode : public rclcpp::Node
{
public:
    ObstacleDetectionNode() : Node("obstacle_detection_node"), processing_(false)
    {
        std::string package_share_dir;
        try {
            package_share_dir = ament_index_cpp::get_package_share_directory("camera");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get package directory: %s", e.what());
            return;
        }
        
        this->declare_parameter("model_cfg", 
            package_share_dir + "/models/yolov3-tiny.cfg");
        this->declare_parameter("model_weights", 
            package_share_dir + "/models/yolov3-tiny.weights");
        this->declare_parameter("class_names", 
            package_share_dir + "/models/coco.names");
        this->declare_parameter("confidence_threshold", 0.5);
        this->declare_parameter("nms_threshold", 0.4);
        this->declare_parameter("input_size", 416);
        
        // Get parameters
        std::string cfg = this->get_parameter("model_cfg").as_string();
        std::string weights = this->get_parameter("model_weights").as_string();
        std::string names_file = this->get_parameter("class_names").as_string();
        conf_threshold_ = this->get_parameter("confidence_threshold").as_double();
        nms_threshold_ = this->get_parameter("nms_threshold").as_double();
        input_size_ = this->get_parameter("input_size").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Model config: %s", cfg.c_str());
        RCLCPP_INFO(this->get_logger(), "Model weights: %s", weights.c_str());

        // Load class names
        std::ifstream ifs(names_file); 
        if (!ifs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open class names file: %s", names_file.c_str());
            return;
        }
        std::string line;
        while (std::getline(ifs, line)) {
            classes_.push_back(line);
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu classes", classes_.size());
        
        // Load YOLO network
        RCLCPP_INFO(this->get_logger(), "Loading YOLO model...");
        try {
            net_ = cv::dnn::readNetFromDarknet(cfg, weights);
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            
            // Get output layer names
            output_names_ = net_.getUnconnectedOutLayersNames();
            
            RCLCPP_INFO(this->get_logger(), "YOLO model loaded successfully");
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YOLO model: %s", e.what());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "YOLO detector node initialized");
    }
    
    // Destructor for clean shutdown
    ~ObstacleDetectionNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down obstacle detection node...");
        
        // Wait for any processing to complete (max 2 seconds)
        auto start = std::chrono::steady_clock::now();
        while (processing_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start);
            if (elapsed.count() > 2) {
                RCLCPP_WARN(this->get_logger(), "Forced shutdown - processing still active");
                break;
            }
        }
        
        // Shutdown subscribers/publishers
        if (image_subscriber_) {
            image_subscriber_.shutdown();
        }
        if (image_publisher_) {
            image_publisher_.shutdown();
        }
        
        RCLCPP_INFO(this->get_logger(), "Shutdown complete");
    }
    
    void initialize()
    {
        // Initialize image transport AFTER the node is managed by shared_ptr
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        
        // Create subscription
        image_subscriber_ = it_->subscribe("/camera/image_raw", 1, 
            std::bind(&ObstacleDetectionNode::imageCallback, this, std::placeholders::_1));
            
        // Create publisher for detected image
        image_publisher_ = it_->advertise("/camera/detected_image", 1);  // Queue size 1, not 10
        
        RCLCPP_INFO(this->get_logger(), "Subscriptions and publishers initialized");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "imageCallback is running");
        // Check if node is shutting down
        if (!rclcpp::ok()) {
            return;
        }
        
        // Skip if already processing (drop frames if too slow)
        if (processing_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Dropping frame - still processing previous frame");
            return;
        }
        
        processing_ = true;
        auto start = std::chrono::high_resolution_clock::now();
        
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            processing_ = false;
            return;
        }
        
        // Check again before heavy processing
        if (!rclcpp::ok()) {
            processing_ = false;
            return;
        }
        
        cv::Mat frame = cv_ptr->image;
        
        // Create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1/255.0, 
                               cv::Size(input_size_, input_size_), 
                               cv::Scalar(0,0,0), true, false);
        
        // Set input to network
        net_.setInput(blob);
        RCLCPP_INFO(this->get_logger(), "Running forward pass");
        // Forward pass (this is the slow part)
        RCLCPP_INFO(this->get_logger(), "Finished forward pass");
        std::vector<cv::Mat> outs;
        net_.forward(outs, output_names_);
        
        // Check if we should still publish
        if (!rclcpp::ok()) {
            processing_ = false;
            return;
        }
        
        // Post-process detections
        postprocess(frame, outs);
        RCLCPP_INFO(this->get_logger(), "Publishing");
        // Publish result
        sensor_msgs::msg::Image::SharedPtr processed_msg = 
            cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        image_publisher_.publish(processed_msg);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "Processing time: %ld ms", duration.count());
        
        processing_ = false;
    }
    
    void postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs)
    {
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        
        for (const auto& out : outs) {
            for (int i = 0; i < out.rows; ++i) {
                const float* data = out.ptr<float>(i);
                cv::Mat scores = out.row(i).colRange(5, out.cols);
                cv::Point classIdPoint;
                double confidence;
                cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                
                if (confidence > conf_threshold_) {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;
                    
                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            }
        }
        
        // Non-maximum suppression
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);
        
        // Draw detections
        for (int idx : indices) {
            cv::Rect box = boxes[idx];
            drawPred(classIds[idx], confidences[idx], box.x, box.y,
                    box.x + box.width, box.y + box.height, frame);
        }
        
        // Display detection count
        std::string count_text = "Detections: " + std::to_string(indices.size());
        cv::putText(frame, count_text, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    }
    
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
    {
        cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), 
                     cv::Scalar(0, 255, 0), 2);
        
        std::string label = cv::format("%.2f", conf);
        if (!classes_.empty() && classId < (int)classes_.size()) {
            label = classes_[classId] + ": " + label;
        }
        
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = std::max(top, labelSize.height);
        cv::rectangle(frame, cv::Point(left, top - labelSize.height),
                     cv::Point(left + labelSize.width, top + baseLine),
                     cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(frame, label, cv::Point(left, top), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);
    }
    
    cv::dnn::Net net_;
    std::vector<std::string> classes_;
    std::vector<std::string> output_names_;
    double conf_threshold_;
    double nms_threshold_;
    int input_size_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher image_publisher_;
    std::atomic<bool> processing_;  // Track if currently processing
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ObstacleDetectionNode>();
    node->initialize();
    
    // Use executor with better shutdown handling
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    // Spin with periodic checks for shutdown
    while (rclcpp::ok()) {
        executor.spin_some(std::chrono::milliseconds(100));
    }
    
    RCLCPP_INFO(node->get_logger(), "Received shutdown signal, cleaning up...");
    
    // Clean shutdown
    executor.remove_node(node);
    node.reset();
    
    rclcpp::shutdown();
    return 0;
}