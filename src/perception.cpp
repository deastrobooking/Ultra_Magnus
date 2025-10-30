#include "ultra_magnus/perception.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace ultra_magnus {

Perception::Perception()
    : min_range_(0.1),
      max_range_(10.0),
      window_size_(5) {
}

std::vector<double> Perception::processLaserScan(
    const sensor_msgs::msg::LaserScan& scan) {
    
    std::vector<double> processed_data;
    processed_data.reserve(scan.ranges.size());
    
    for (const auto& range : scan.ranges) {
        // Filter out invalid readings
        if (std::isfinite(range) && range >= min_range_ && range <= max_range_) {
            processed_data.push_back(range);
        } else {
            // Replace invalid readings with max range
            processed_data.push_back(max_range_);
        }
    }
    
    // Apply noise filtering
    return filterNoise(processed_data);
}

double Perception::detectFrontObstacle(
    const sensor_msgs::msg::LaserScan& scan) {
    
    if (scan.ranges.empty()) {
        return max_range_;
    }
    
    // Take center portion of scan (assuming front-facing)
    size_t center = scan.ranges.size() / 2;
    size_t window = scan.ranges.size() / 10; // 10% of scan around center
    
    double min_distance = max_range_;
    
    for (size_t i = center - window; i <= center + window && i < scan.ranges.size(); ++i) {
        if (std::isfinite(scan.ranges[i]) && 
            scan.ranges[i] >= min_range_ && 
            scan.ranges[i] <= max_range_) {
            min_distance = std::min(min_distance, static_cast<double>(scan.ranges[i]));
        }
    }
    
    return min_distance;
}

std::vector<double> Perception::filterNoise(
    const std::vector<double>& raw_data) {
    
    if (raw_data.size() < window_size_) {
        return raw_data;
    }
    
    std::vector<double> filtered_data;
    filtered_data.reserve(raw_data.size());
    
    // Moving average filter
    for (size_t i = 0; i < raw_data.size(); ++i) {
        size_t start = (i >= window_size_ / 2) ? i - window_size_ / 2 : 0;
        size_t end = std::min(i + window_size_ / 2 + 1, raw_data.size());
        
        double sum = std::accumulate(raw_data.begin() + start, 
                                     raw_data.begin() + end, 
                                     0.0);
        double avg = sum / (end - start);
        filtered_data.push_back(avg);
    }
    
    return filtered_data;
}

} // namespace ultra_magnus
