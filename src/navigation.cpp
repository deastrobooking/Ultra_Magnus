#include "ultra_magnus/navigation.hpp"
#include <cmath>

namespace ultra_magnus {

Navigation::Navigation()
    : goal_set_(false),
      goal_tolerance_(0.1),
      max_linear_velocity_(1.0),
      max_angular_velocity_(1.0) {
}

bool Navigation::setGoal(const geometry_msgs::msg::Pose& goal) {
    goal_ = goal;
    goal_set_ = true;
    return true;
}

geometry_msgs::msg::Twist Navigation::computeVelocityCommand(
    const geometry_msgs::msg::Pose& current_pose) {
    
    geometry_msgs::msg::Twist cmd;
    
    if (!goal_set_) {
        return cmd; // Return zero velocity if no goal set
    }
    
    current_pose_ = current_pose;
    
    // Calculate distance to goal
    double dx = goal_.position.x - current_pose.position.x;
    double dy = goal_.position.y - current_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Check if goal reached
    if (distance < goal_tolerance_) {
        return cmd; // Return zero velocity at goal
    }
    
    // Calculate angle to goal
    double angle_to_goal = std::atan2(dy, dx);
    
    // Simple proportional control
    cmd.linear.x = std::min(max_linear_velocity_, distance);
    cmd.angular.z = std::min(max_angular_velocity_, std::max(-max_angular_velocity_, angle_to_goal));
    
    return cmd;
}

bool Navigation::isGoalReached() const {
    if (!goal_set_) {
        return false;
    }
    
    double distance = getDistanceToGoal();
    return distance < goal_tolerance_;
}

double Navigation::getDistanceToGoal() const {
    if (!goal_set_) {
        return std::numeric_limits<double>::max();
    }
    
    double dx = goal_.position.x - current_pose_.position.x;
    double dy = goal_.position.y - current_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace ultra_magnus
