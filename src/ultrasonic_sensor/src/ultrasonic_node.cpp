#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64.hpp>
#include "libHCSR04.hpp"

class UltrasonicSensorNode : public rclcpp::Node {
public:
    UltrasonicSensorNode() : Node("ultrasonic_sensor_node") {
        // Declare parameters with default values
        // Assigning value to trigger pin of this class
        this->declare_parameter<int>("trigger_pin", 18);
        this->declare_parameter<int>("echo_pin", 24);
        this->declare_parameter<double>("publish_rate", 10.0);  // Hz
        this->declare_parameter<double>("max_range", 4.0);      // meters
        this->declare_parameter<double>("min_range", 0.02);     // meters
        this->declare_parameter<std::string>("frame_id", "ultrasonic_frame");
        
        // Get parameters
        int trigger_pin = this->get_parameter("trigger_pin").as_int();
        int echo_pin = this->get_parameter("echo_pin").as_int();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        min_range_ = this->get_parameter("min_range").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // Initialize sensor
        if (!ultrasonic_sensor_.init(trigger_pin, echo_pin)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize HC-SR04 sensor");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), 
                    "HC-SR04 initialized with trigger pin %d, echo pin %d", 
                    trigger_pin, echo_pin);
        
        // Create publishers
        // ~/range that i need to refer to this publisher as ultrasonic_sensor/range from a different package
        range_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("~/range", 10);
        distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("~/distance", 10);
        
        // Create timer
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            timer_period, 
            std::bind(&UltrasonicSensorNode::timer_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Ultrasonic sensor node started at %.1f Hz", publish_rate);
    }
    
    ~UltrasonicSensorNode() {
        ultrasonic_sensor_.cleanup();
    }

private:
    void timer_callback() {
        // Periodically runs to publish sensor data
        double distance_cm = ultrasonic_sensor_.distance();
        
        if (distance_cm < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Failed to get distance measurement");
            return;
        }
        
        double distance_m = distance_cm / 100.0;  // Convert cm to meters
        
        // Publish Range message (standard ROS sensor message)
        auto range_msg = sensor_msgs::msg::Range();
        range_msg.header.stamp = this->now();
        range_msg.header.frame_id = frame_id_;
        range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        range_msg.field_of_view = 0.26;  // ~15 degrees in radians
        range_msg.min_range = min_range_;
        range_msg.max_range = max_range_;
        range_msg.range = distance_m;
        
        range_publisher_->publish(range_msg);
        
        // Also publish simple distance for easy debugging
        auto distance_msg = std_msgs::msg::Float64();
        distance_msg.data = distance_cm;
        distance_publisher_->publish(distance_msg);
        
        // Log occasionally
        static int counter = 0;
        if (++counter % 50 == 0) {  // Every ~5 seconds at 10Hz
            RCLCPP_INFO(this->get_logger(), "Distance: %.2f cm", distance_cm);
        }
    }
    
    HCSR04 ultrasonic_sensor_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double max_range_;
    double min_range_;
    std::string frame_id_;
};

int main(int argc, char** argv) {
    //initializes communication
    rclcpp::init(argc, argv);
    
    try {
        //Dynamically creates object then returns a sharped pointer, object is of type UltrasonicSensorNode
        //auto means type of variable will be deduced from its initializer
        auto node = std::make_shared<UltrasonicSensorNode>();
        //Allows node to keep running and check for events on subscribed topics and service calls
        //Handles subscription callbacks and service requests, but this node only publishes
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
