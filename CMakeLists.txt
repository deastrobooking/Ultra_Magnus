# Minimum CMake version
cmake_minimum_required(VERSION 3.16)
project(ultra_magnus VERSION 1.0.0 LANGUAGES CXX)

# Modern C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Essential flags for robotics
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -O2")

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Optional packages
find_package(Eigen3 QUIET)
find_package(PCL QUIET)
find_package(OpenCV QUIET)

# Include directories
include_directories(include)

# Add your library targets
add_library(robot_core 
    src/controller.cpp
    src/navigation.cpp
    src/perception.cpp
)

# Link dependencies for ROS 2
ament_target_dependencies(robot_core
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
)

# Link optional dependencies
if(Eigen3_FOUND)
    target_link_libraries(robot_core Eigen3::Eigen)
endif()

if(PCL_FOUND)
    target_link_libraries(robot_core ${PCL_LIBRARIES})
endif()

if(OpenCV_FOUND)
    target_link_libraries(robot_core ${OpenCV_LIBS})
endif()

# Executable
add_executable(robot_main src/main.cpp)
target_link_libraries(robot_main robot_core)
ament_target_dependencies(robot_main
    rclcpp
)

# Install targets
install(TARGETS
    robot_core
    robot_main
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    
    ament_add_gtest(test_controller test/test_controller.cpp)
    target_link_libraries(test_controller robot_core)
    ament_target_dependencies(test_controller rclcpp)
endif()

ament_package()
