#include "ultra_magnus/controller.hpp"
#include <algorithm>
#include <cmath>

namespace ultra_magnus {

RobotController::RobotController()
    : min_safe_distance_(0.5),
      max_speed_(1.0) {
    // Initialize command buffer
    for (auto& cmd : command_buffer_) {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
    }
}

geometry_msgs::msg::Twist RobotController::computeCommand(
    const std::vector<double>& obstacles) {
    
    geometry_msgs::msg::Twist command;
    
    // Check emergency stop
    if (emergency_stop_active_.load(std::memory_order_acquire)) {
        return command; // Return zero velocity
    }
    
    // Check for collisions
    if (checkCollision(obstacles)) {
        emergencyStop();
        return command;
    }
    
    // Find minimum obstacle distance
    double min_distance = std::numeric_limits<double>::max();
    if (!obstacles.empty()) {
        min_distance = *std::min_element(obstacles.begin(), obstacles.end());
    }
    
    // Compute safe velocity
    command = computeSafeVelocity(min_distance);
    
    // Store in buffer (if space available)
    if (buffer_index_ < command_buffer_.size()) {
        command_buffer_[buffer_index_++] = command;
    }
    
    return command;
}

void RobotController::updateSafetyParams(double min_distance, double max_speed) {
    std::lock_guard<std::mutex> lock(params_mutex_);
    min_safe_distance_ = min_distance;
    max_speed_ = max_speed;
}

void RobotController::emergencyStop() {
    emergency_stop_active_.store(true, std::memory_order_release);
}

bool RobotController::isSafe() const {
    return !emergency_stop_active_.load(std::memory_order_acquire);
}

bool RobotController::checkCollision(const std::vector<double>& obstacles) const {
    std::lock_guard<std::mutex> lock(params_mutex_);
    
    for (const auto& distance : obstacles) {
        if (distance < min_safe_distance_) {
            return true; // Collision detected
        }
    }
    return false;
}

geometry_msgs::msg::Twist RobotController::computeSafeVelocity(
    double min_obstacle_distance) {
    
    geometry_msgs::msg::Twist velocity;
    std::lock_guard<std::mutex> lock(params_mutex_);
    
    // Simple proportional control based on obstacle distance
    if (min_obstacle_distance > min_safe_distance_) {
        double safety_factor = (min_obstacle_distance - min_safe_distance_) / 
                              (min_safe_distance_ * 2.0);
        safety_factor = std::min(1.0, std::max(0.0, safety_factor));
        
        velocity.linear.x = max_speed_ * safety_factor;
    }
    
    return velocity;
}

} // namespace ultra_magnus
