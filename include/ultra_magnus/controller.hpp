#ifndef ULTRA_MAGNUS_CONTROLLER_HPP
#define ULTRA_MAGNUS_CONTROLLER_HPP

#include <memory>
#include <vector>
#include <array>
#include <atomic>
#include <mutex>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace ultra_magnus {

/**
 * @brief Real-time safe robot controller
 * 
 * Implements safety-critical control with pre-allocated memory
 * and thread-safe data handling.
 */
class RobotController {
public:
    RobotController();
    ~RobotController() = default;

    // Delete copy constructor and assignment operator
    RobotController(const RobotController&) = delete;
    RobotController& operator=(const RobotController&) = delete;

    // Default move constructor and assignment operator
    RobotController(RobotController&&) = default;
    RobotController& operator=(RobotController&&) = default;

    /**
     * @brief Compute control command based on sensor data
     * @param obstacles Vector of distance measurements
     * @return Twist command for robot motion
     */
    geometry_msgs::msg::Twist computeCommand(const std::vector<double>& obstacles);

    /**
     * @brief Update safety parameters
     * @param min_distance Minimum safe distance to obstacles
     * @param max_speed Maximum allowed speed
     */
    void updateSafetyParams(double min_distance, double max_speed);

    /**
     * @brief Emergency stop the robot
     */
    void emergencyStop();

    /**
     * @brief Check if robot is in safe state
     */
    bool isSafe() const;

private:
    // Pre-allocated command buffer for real-time safety
    static constexpr size_t COMMAND_BUFFER_SIZE = 1000;
    std::array<geometry_msgs::msg::Twist, COMMAND_BUFFER_SIZE> command_buffer_;
    size_t buffer_index_ = 0;

    // Thread-safe parameters
    mutable std::mutex params_mutex_;
    double min_safe_distance_;
    double max_speed_;
    std::atomic<bool> emergency_stop_active_{false};

    // Internal helper methods
    bool checkCollision(const std::vector<double>& obstacles) const;
    geometry_msgs::msg::Twist computeSafeVelocity(double min_obstacle_distance);
};

/**
 * @brief Ring buffer for sensor data storage
 * 
 * Thread-safe ring buffer implementation for efficient
 * real-time sensor data management.
 */
template<typename T, size_t N>
class SensorRingBuffer {
private:
    std::array<T, N> buffer_;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
    mutable std::mutex mutex_;

public:
    bool push(const T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (count_ >= N) return false;
        buffer_[head_] = item;
        head_ = (head_ + 1) % N;
        ++count_;
        return true;
    }
    
    bool pop(T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (count_ == 0) return false;
        item = buffer_[tail_];
        tail_ = (tail_ + 1) % N;
        --count_;
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_ == 0;
    }
};

} // namespace ultra_magnus

#endif // ULTRA_MAGNUS_CONTROLLER_HPP
