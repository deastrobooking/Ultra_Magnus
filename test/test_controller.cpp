#include <gtest/gtest.h>
#include "ultra_magnus/controller.hpp"
#include <geometry_msgs/msg/twist.hpp>

/**
 * @brief Test fixture for RobotController
 */
class ControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller_ = std::make_unique<ultra_magnus::RobotController>();
        controller_->updateSafetyParams(0.5, 1.0);
    }
    
    void TearDown() override {
        controller_.reset();
    }
    
    std::unique_ptr<ultra_magnus::RobotController> controller_;
};

TEST_F(ControllerTest, EmergencyStopOnCloseObstacle) {
    // Test with close obstacles
    std::vector<double> close_obstacles = {0.1, 0.2, 0.3};
    auto command = controller_->computeCommand(close_obstacles);
    
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
    EXPECT_FALSE(controller_->isSafe());
}

TEST_F(ControllerTest, NormalOperationWithClearPath) {
    // Test with clear path
    std::vector<double> clear_path = {1.0, 2.0, 3.0, 4.0};
    auto command = controller_->computeCommand(clear_path);
    
    EXPECT_GT(command.linear.x, 0.0);
    EXPECT_TRUE(controller_->isSafe());
}

TEST_F(ControllerTest, SafetyParameterUpdate) {
    // Update safety parameters
    controller_->updateSafetyParams(1.0, 2.0);
    
    // Test with obstacle at 0.8m (should be safe with new params)
    std::vector<double> obstacles = {0.8, 1.5, 2.0};
    auto command = controller_->computeCommand(obstacles);
    
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST_F(ControllerTest, EmergencyStopManual) {
    // Manually trigger emergency stop
    controller_->emergencyStop();
    EXPECT_FALSE(controller_->isSafe());
    
    // Command should be zero even with clear path
    std::vector<double> clear_path = {5.0, 6.0, 7.0};
    auto command = controller_->computeCommand(clear_path);
    
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST_F(ControllerTest, VelocityScalingWithDistance) {
    // Test velocity scaling based on obstacle distance
    controller_->updateSafetyParams(0.5, 1.0);
    
    std::vector<double> far_obstacles = {5.0, 6.0, 7.0};
    auto cmd_far = controller_->computeCommand(far_obstacles);
    
    std::vector<double> mid_obstacles = {1.0, 1.5, 2.0};
    auto cmd_mid = controller_->computeCommand(mid_obstacles);
    
    // Far obstacles should allow higher velocity than mid-range obstacles
    EXPECT_GE(cmd_far.linear.x, cmd_mid.linear.x);
}

/**
 * @brief Test fixture for SensorRingBuffer
 */
class RingBufferTest : public ::testing::Test {
protected:
    ultra_magnus::SensorRingBuffer<double, 5> buffer_;
};

TEST_F(RingBufferTest, PushAndPop) {
    double value;
    
    // Test push
    EXPECT_TRUE(buffer_.push(1.0));
    EXPECT_TRUE(buffer_.push(2.0));
    EXPECT_TRUE(buffer_.push(3.0));
    EXPECT_EQ(buffer_.size(), 3);
    
    // Test pop
    EXPECT_TRUE(buffer_.pop(value));
    EXPECT_DOUBLE_EQ(value, 1.0);
    EXPECT_EQ(buffer_.size(), 2);
    
    EXPECT_TRUE(buffer_.pop(value));
    EXPECT_DOUBLE_EQ(value, 2.0);
}

TEST_F(RingBufferTest, OverflowProtection) {
    // Fill buffer
    for (int i = 0; i < 5; ++i) {
        EXPECT_TRUE(buffer_.push(static_cast<double>(i)));
    }
    
    // Try to overflow
    EXPECT_FALSE(buffer_.push(100.0));
    EXPECT_EQ(buffer_.size(), 5);
}

TEST_F(RingBufferTest, EmptyBuffer) {
    double value;
    
    EXPECT_TRUE(buffer_.empty());
    EXPECT_FALSE(buffer_.pop(value));
    
    buffer_.push(1.0);
    EXPECT_FALSE(buffer_.empty());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
