#ifndef ULTRA_MAGNUS_NAVIGATION_HPP
#define ULTRA_MAGNUS_NAVIGATION_HPP

#include <memory>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace ultra_magnus {

/**
 * @brief Navigation system for path planning and execution
 */
class Navigation {
public:
    Navigation();
    ~Navigation() = default;

    /**
     * @brief Set goal position for navigation
     * @param goal Target pose
     * @return true if goal is reachable
     */
    bool setGoal(const geometry_msgs::msg::Pose& goal);

    /**
     * @brief Compute velocity command to reach goal
     * @param current_pose Current robot pose
     * @return Velocity command
     */
    geometry_msgs::msg::Twist computeVelocityCommand(
        const geometry_msgs::msg::Pose& current_pose);

    /**
     * @brief Check if goal has been reached
     */
    bool isGoalReached() const;

    /**
     * @brief Get distance to goal
     */
    double getDistanceToGoal() const;

private:
    geometry_msgs::msg::Pose goal_;
    geometry_msgs::msg::Pose current_pose_;
    bool goal_set_;
    
    double goal_tolerance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
};

} // namespace ultra_magnus

#endif // ULTRA_MAGNUS_NAVIGATION_HPP
