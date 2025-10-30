#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "ultra_magnus/controller.hpp"
#include "ultra_magnus/navigation.hpp"
#include "ultra_magnus/perception.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief Main ROS 2 node for Ultra Magnus robot controller
 */
class UltraMagnusNode : public rclcpp::Node {
private:
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    // Timer for control loop
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Core components
    std::unique_ptr<ultra_magnus::RobotController> controller_;
    std::unique_ptr<ultra_magnus::Navigation> navigation_;
    std::unique_ptr<ultra_magnus::Perception> perception_;
    
    // Latest sensor data
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;

public:
    UltraMagnusNode() : Node("ultra_magnus_controller") {
        // Declare parameters
        this->declare_parameter<double>("max_speed", 1.0);
        this->declare_parameter<double>("safety_distance", 0.5);
        this->declare_parameter<int>("control_frequency", 100);
        
        // Initialize components
        controller_ = std::make_unique<ultra_magnus::RobotController>();
        navigation_ = std::make_unique<ultra_magnus::Navigation>();
        perception_ = std::make_unique<ultra_magnus::Perception>();
        
        // Get parameters and update controller
        double max_speed = this->get_parameter("max_speed").as_double();
        double safety_dist = this->get_parameter("safety_distance").as_double();
        controller_->updateSafetyParams(safety_dist, max_speed);
        
        // Create subscribers
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&UltraMagnusNode::lidarCallback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&UltraMagnusNode::odomCallback, this, std::placeholders::_1));
        
        // Create publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Create control timer
        int control_freq = this->get_parameter("control_frequency").as_int();
        auto period = std::chrono::milliseconds(1000 / control_freq);
        control_timer_ = this->create_wall_timer(
            period,
            std::bind(&UltraMagnusNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Ultra Magnus controller initialized");
        RCLCPP_INFO(this->get_logger(), "Max speed: %.2f m/s", max_speed);
        RCLCPP_INFO(this->get_logger(), "Safety distance: %.2f m", safety_dist);
        RCLCPP_INFO(this->get_logger(), "Control frequency: %d Hz", control_freq);
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
        
        // Process scan data
        auto obstacles = perception_->processLaserScan(*msg);
        
        // Check for close obstacles
        double min_distance = perception_->detectFrontObstacle(*msg);
        double safety_dist = this->get_parameter("safety_distance").as_double();
        
        if (min_distance < safety_dist) {
            RCLCPP_WARN(this->get_logger(), 
                       "Close obstacle detected at %.2f m!", min_distance);
            controller_->emergencyStop();
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
    }
    
    void controlLoop() {
        if (!latest_scan_) {
            // No sensor data yet
            return;
        }
        
        // Process latest scan
        auto obstacles = perception_->processLaserScan(*latest_scan_);
        
        // Compute control command
        auto command = controller_->computeCommand(obstacles);
        
        // Publish command
        cmd_vel_pub_->publish(command);
        
        // Log status periodically
        static int counter = 0;
        if (++counter % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "Status: %s, Linear vel: %.2f m/s", 
                       controller_->isSafe() ? "SAFE" : "EMERGENCY_STOP",
                       command.linear.x);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<UltraMagnusNode>();
    
    RCLCPP_INFO(node->get_logger(), "Ultra Magnus starting...");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
