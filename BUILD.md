# Ultra Magnus Build Instructions

## Prerequisites

### Required
- Ubuntu 22.04 LTS or later
- ROS 2 Humble or later
- CMake 3.16+
- GCC 9+ or Clang 10+ (C++17 support required)

### Optional Dependencies
- Eigen3 (for advanced math operations)
- PCL (Point Cloud Library - for 3D perception)
- OpenCV (for vision processing)

## Installation

### 1. Install ROS 2

Follow the official ROS 2 installation guide:
https://docs.ros.org/en/humble/Installation.html

### 2. Create ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. Clone Repository

```bash
git clone https://github.com/deastrobooking/Ultra_Magnus.git
cd ~/ros2_ws
```

### 4. Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the Project

```bash
cd ~/ros2_ws
colcon build --packages-select ultra_magnus
source install/setup.bash
```

## Running the Robot Controller

### Basic Launch

```bash
ros2 launch ultra_magnus robot.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch ultra_magnus robot.launch.py \
    max_speed:=2.0 \
    safety_distance:=1.0 \
    control_frequency:=50
```

### Run with Configuration File

```bash
ros2 run ultra_magnus robot_main --ros-args \
    --params-file src/ultra_magnus/config/params.yaml
```

## Testing

### Run Unit Tests

```bash
cd ~/ros2_ws
colcon test --packages-select ultra_magnus
colcon test-result --verbose
```

### Run Specific Test

```bash
cd ~/ros2_ws/build/ultra_magnus
./test_controller
```

## Development Workflow

### Format Code

```bash
# Install clang-format if not available
sudo apt install clang-format

# Format all C++ files
find src include -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
```

### Static Analysis

```bash
# Install clang-tidy if not available
sudo apt install clang-tidy

# Run static analysis
clang-tidy src/*.cpp -- -I include -std=c++17
```

### Debug Build

```bash
colcon build --packages-select ultra_magnus --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Troubleshooting

### Missing Dependencies

If you encounter missing dependency errors:

```bash
sudo apt update
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-nav-msgs
```

### Build Errors

Clean and rebuild:

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select ultra_magnus
```

## Project Structure

```
ultra_magnus/
├── CMakeLists.txt          # Build configuration
├── package.xml             # ROS 2 package metadata
├── include/                # Header files
│   └── ultra_magnus/
│       ├── controller.hpp
│       ├── navigation.hpp
│       └── perception.hpp
├── src/                    # Implementation files
│   ├── controller.cpp
│   ├── navigation.cpp
│   ├── perception.cpp
│   └── main.cpp
├── launch/                 # Launch files
│   └── robot.launch.py
├── config/                 # Configuration files
│   └── params.yaml
├── test/                   # Unit tests
│   └── test_controller.cpp
└── scripts/                # Utility scripts
    └── calibration.py
```

## Contributing

Please follow the coding standards outlined in the main README.md and ensure all tests pass before submitting pull requests.

## License

Apache-2.0
