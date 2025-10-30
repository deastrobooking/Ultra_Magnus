#ifndef ULTRA_MAGNUS_PERCEPTION_HPP
#define ULTRA_MAGNUS_PERCEPTION_HPP

#include <memory>
#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace ultra_magnus {

/**
 * @brief Perception system for sensor data processing
 */
class Perception {
public:
    Perception();
    ~Perception() = default;

    /**
     * @brief Process laser scan data
     * @param scan Laser scan message
     * @return Vector of obstacle distances
     */
    std::vector<double> processLaserScan(
        const sensor_msgs::msg::LaserScan& scan);

    /**
     * @brief Detect obstacles in front of robot
     * @param scan Laser scan message
     * @return Minimum distance to obstacle
     */
    double detectFrontObstacle(
        const sensor_msgs::msg::LaserScan& scan);

    /**
     * @brief Filter sensor noise
     * @param raw_data Raw sensor readings
     * @return Filtered sensor data
     */
    std::vector<double> filterNoise(
        const std::vector<double>& raw_data);

private:
    double min_range_;
    double max_range_;
    size_t window_size_;
};

} // namespace ultra_magnus

#endif // ULTRA_MAGNUS_PERCEPTION_HPP
