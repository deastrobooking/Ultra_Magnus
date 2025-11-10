# Ultra_Magnus
RTOS &amp; ROS2 Software for Advanced Robotics
# C++ Robotics Developer Starter Guide
- [Google Scholar Robitcs TOp 100](https://scholar.google.com/citations?view_op=top_venues&hl=en&vq=eng_robotics)
## Nvidia 
- [Isaac Lab Github](https://github.com/isaac-sim/IsaacLab)
- [Isaac Lab Docs](https://isaac-sim.github.io/IsaacLab/main/index.html)
- [Isaac Lab](https://developer.nvidia.com/isaac/lab) 
## Table of Contents
1. [Modern C++ Fundamentals](#1-modern-c-fundamentals)
2. [Robotics-Specific C++ Patterns](#2-robotics-specific-c-patterns)
3. [Essential Libraries & Tools](#3-essential-libraries--tools)
4. [ROS 2 Integration](#4-ros-2-integration)
5. [Project Structure & Best Practices](#5-project-structure--best-practices)
6. [Learning Path](#6-learning-path)

***

- **Overview**
  - Mechanical Design, Kinematics, Dynamics, CAD Modeling
  - Electronics: Sensors, Actuators, Motor Control, PCB Design
  - Control Systems: PID, State-space, MPC, Whole-body control
  - ROS 2, Gazebo Ignition Simulation practice
  - Real robot programming: Unitree G1 & Poppy Robot 

***

### Prerequisite Learning Resources

#### Mathematics & Physics Foundations
- [Basic Maths for Robotics (The Construct)](https://app.theconstruct.ai/courses/53) 
- [Engineering Mechanics: Statics (Coursera, Georgia Tech)](https://www.coursera.org/learn/engineering-mechanics-statics) 

#### Programming Essentials
- [C++ Essentials for Robotics (The Construct)](https://app.theconstruct.ai/courses/59) 
- [Python Essentials for Robotics (The Construct)](https://app.theconstruct.ai/courses/58) 
- Example: [Robotics Algorithms in C++ (Online Book)](https://www.roboteq.com/index.php/docman/miscellaneous/374-robotics-algorithms-in-c-programming/file) 

#### Linux & ROS
- [Linux Essentials for Robotics (The Construct)](https://app.theconstruct.ai/courses/40) 
- [ROS 2 Training (The Construct)](https://www.theconstruct.ai/ros-developer/) 

#### Basic Electronics
- [Introduction to Electronics (Coursera)](https://www.coursera.org/learn/electronics) 
- [PCB Design Guide (Altium)](https://resources.altium.com/p/pcb-design-guide) 

***

### Simulation & Robot Development

#### Robot Operating System (ROS 2)
- [ROS 2 Official Documentation](https://docs.ros.org/en/rolling/index.html) 
- [Open-RMF Training (The Construct)](https://www.theconstruct.ai/open-rmf-training/) 

#### Gazebo Ignition Simulation
- [Gazebo Official Simulator](https://gazebosim.org/) 
- [Simulated Robots with Gazebo (Construct Course)](https://www.theconstruct.ai/robotics-simulation/) 

#### Humanoid Robot Kits
- [Poppy Robot Project](https://www.poppy-project.org/en/) 
- [Unitree Robotics (Unitree G1 Details)](https://www.unitree.com/products/unitree-g1/) 

***

### Advanced Control Systems and AI

#### Humanoid Control Tutorials
- [Modern Robotics: Mechanics, Planning, and Control (Northwestern Textbook)](http://modernrobotics.org/) 
- [Robotics: State Estimation and Control (MIT OCW)](https://ocw.mit.edu/courses/6-832-underactuated-robotics-spring-2020/) 
- [AI for Robotics (Coursera, Georgia Tech)](https://www.coursera.org/learn/ai-for-robotics) 

#### Full Robot Projects & Monitoring
- [Behavior Trees (ROS Wiki)](https://wiki.ros.org/behavior_tree) 
- [Visual Navigation Algorithms (OpenCV & SLAM)](https://opencv.org/) 

***

### Electronics and Power

- [Motor Control Theory (Texas Instruments)](https://www.ti.com/motor-drivers/overview.html) 
- [Battery Management Systems (Analog Devices)](https://www.analog.com/en/applications/technology/battery-management.html) 

***

### Support, Community & Events

- [The Construct Robotics Forum](https://get-help.theconstruct.ai/) 
- [ROS Developers Podcast](https://www.theconstruct.ai/ros-developers-podcast/) 
- [Robotics Developers Day & ROS Awards](https://www.theconstruct.ai/events/) 

***

### Recommended Reference Books

| Title                                                             | Author(s)                      | Focus Area                                               |
|-------------------------------------------------------------------|-------------------------------|---------------------------------------------------------|
| Modern Robotics: Mechanics, Planning, and Control                 | Lynch & Park                  | Kinematics, planning, control theory             |
| Introduction to Autonomous Robots                                 | Burgard, Siegwart, Nourbakhsh | Fundamentals of mobile robots                    |
| Robotics: Control, Sensing, Vision, and Intelligence              | Fu, Gonzalez, Lee             | Sensors, vision, controls                        |
| Humanoid Robotics: A Reference                                    | Goswami & Vadakkepat          | Comprehensive humanoid systems                   |

***

### Industry Tools and Open Source Resources

- [Open Robotics (ROS & Gazebo Providers)](https://www.openrobotics.org/) 
- [GitHub - ROS2 Awesome List](https://github.com/fkromer/awesome-ros2) 
- [Simulink Robotics Control Demos (MathWorks)](https://www.mathworks.com/solutions/robotics.html) 
- [KiCad – Free PCB Design Software](https://www.kicad.org/) 
- [FreeCAD – Open-Source CAD Tool](https://www.freecadweb.org/) 

***


### 1. Curated Lists & Roadmaps
- **Awesome Embedded**: Curated frameworks, libraries, and tools for embedded development (from C/Rust libraries to RTOS and MicroPython) — broadly applicable to embedded robotics.  
  [GitHub: memfault/awesome-embedded](https://github.com/memfault/awesome-embedded)  
- **Awesome Humanoid Learning**: Focused on research, simulation models, papers, and community resources for bipedal/humanoid robots. Includes models (URDF, MJCF, COLLADA), references to famous robots (Boston Dynamics Atlas, SoftBank NAO, etc.), and controller resources.  
  [DeepWiki about this repo](https://deepwiki.com/jonyzhang2023/awesome-humanoid-learning)
- **Embedded Engineering Roadmap**: A learning roadmap for mastering embedded systems for robotics, with books, courses, and projects for low-level programming and hardware.  
  [GitHub Topics: embedded-systems](https://github.com/topics/embedded-systems)

### 2. Top Open-Source Platforms for Humanoid Robotics
- **Robot Operating System (ROS/ROS2)**: The central open-source middleware for robotics (sensors, motor control, embedded comms); used in many humanoid platforms and educational robots.  
- **Poppy Project**: Open-source humanoid robot project offering mechanical designs, software, and simulation—all suitable for embedded development and research.  
  [ThinkRobotics top open-source platforms](https://thinkrobotics.com/blogs/learn/top-7-open-source-robotics-platforms)

### 3. Simulation & Middleware Libraries
- **Webots**: Open-source robotics simulation environment supporting humanoid models and embedded robot controller integration.  
- **Unity Robotics Hub**: Examples and tools for using Unity in robotics simulation and controller visualization (including embedded-centric projects).
  [Awesome Robotics Libraries](http://jslee02.github.io/awesome-robotics-libraries/)

### 4. Recent Industry Contributions
- **EngineAI’s RL Workspace**: Modular, open-source reinforcement learning environments and ROS2 infrastructure for real-world deployment of humanoid and legged robots.  
  [Press release about EngineAI’s open-source tools](https://www.prnewswire.com/news-releases/engineai-releases-comprehensive-open-source-resources-to-accelerate-robotics-development-302481108.html)

### 5. Embedded System Libraries for Robotics
- **MicroPython**: Python for microcontrollers/embedded controllers in robots.  
- **Zephyr RTOS**: Scalable, secure real-time operating system, popular in embedded robotics and microcontroller-based humanoids.  
  [LibHunt embedded topic](https://www.libhunt.com/topic/embedded)

#### Bonus: Aggregator Pages
- **AIBase Embedded Topic** and **LibHunt Embedded Projects**: Track trending and popular embedded systems repositories, including those relevant for robotics.  
  [Popular embedded repos](https://www.aibase.com/repos/topic/embedded-systems)

---
1. [GitHub - memfault/awesome-embedded](https://github.com/memfault/awesome-embedded)
2. [Awesome Embedded Resources for Developers (Adafruit blog)](https://blog.adafruit.com/2023/06/27/awesome-embedded-resources-for-developers-awesome-software-embedded/)
3. [GitHub Topics: embedded-systems](https://github.com/topics/embedded-systems)
4. [jonyzhang2023/awesome-humanoid-learning (DeepWiki)](https://deepwiki.com/jonyzhang2023/awesome-humanoid-learning)
5. [Top 7 Open-Source Robotics Platforms – ThinkRobotics.com](https://thinkrobotics.com/blogs/learn/top-7-open-source-robotics-platforms)
6. [EngineAI Open-Source Announcement](https://www.prnewswire.com/news-releases/engineai-releases-comprehensive-open-source-resources-to-accelerate-robotics-development-302481108.html)
7. [Awesome Robotics Libraries](http://jslee02.github.io/awesome-robotics-libraries/)
8. [LibHunt: Embedded Open-Source Projects](https://www.libhunt.com/topic/embedded)
9. [AIBase Embedded Repositories](https://www.aibase.com/repos/topic/embedded-systems)


### 1. Official ROS 2 and Core C++ Libraries
- **ros2/ros2:** This is the primary entry point for all version 2 development. It includes installation guides, build tools, and meta-packages. Essential for setting up a ROS 2 workspace and understanding core features[[1]](https://roboticsbiz.com/ros-2-essential-resources-and-repositories-for-developers/)[[2]](https://github.com/ros2/ros2).
- **ros2/rclcpp:** The standard C++ client library for ROS 2. It covers core message passing, service definitions, action handling, and node lifecycle management. Any custom C++ node will usually be built using `rclcpp`[[3]](https://github.com/ros2).
- **ros/ros_comm:** For ROS 1 users, this repository contains the C++ (`roscpp`) and Python (`rospy`) client libraries and the main communications packages. While ROS 2 is preferred for new projects, ROS 1 still powers many existing humanoid robots.[[4]](https://github.com/ros)

### 2. Humanoid/Legged Robot Projects
- **Poppy Project:** The open-source Poppy platform offers hardware, Python/C++, and ROS-based control for humanoid robots. Designed for education and research, it’s highly modular and customizable for humanoid applications[[5]](https://thinkrobotics.com/blogs/learn/top-7-open-source-robotics-platforms).
- **TurtleBot3 (ROBOTIS-GIT/turtlebot3):** While TurtleBot3 is more wheeled-robot focused, its ROS packages are widely reused in larger legged and humanoid robots for navigation, SLAM, and sensor integration. Their C++ code can be extended for complex robotics systems[[6]](https://github.com/fkromer/awesome-ros2).

### 3. Essential ROS Packages and Tools
- **robot_state_publisher:** Publishes the state of a robot's joints using C++ and the `tf` transform library, an essential tool for any multi-joint or humanoid robot visualization.[[4]](https://github.com/ros)
- **kdl_parser:** Parses URDF models and constructs a KDL (Kinematics and Dynamics Library) tree, widely used for kinematics and motion planning in humanoid robots.[[4]](https://github.com/ros)
- **urdfdom:** Universal Robot Description Format (URDF) parser in C++ for handling robot models.[[4]](https://github.com/ros)

### 4. Simulation and Visualization
- **rviz (ros2/rviz):** 3D visualization for robot data—joint states, sensors, environment mapping—critical for developing and debugging humanoid movements and sensor integration in C++ nodes.[[3]](https://github.com/ros2)
- **Gazebo Integration:** Repositories like `gazebo_ros_pkgs` enable simulation of humanoid robots running ROS nodes, with support for multi-joint actuation and sensor feedback in simulation.

### 5. Curated Lists and Community Resources
- **fkromer/awesome-ros2:** A curated list of ROS 2 resources, libraries, and demonstration packages. Valuable for finding specialized control packages, benchmarking tools, and cross-compilation guides for running C++ humanoid code on embedded platforms[[6]](https://github.com/fkromer/awesome-ros2).
- **robot-operating-system Topic on GitHub:** Explore trending repositories tagged with `robot-operating-system`. This helps discover new and actively maintained C++ ROS packages, including those focused on advanced robotics applications[[7]](https://github.com/topics/robot-operating-system)[[8]](https://github.com/topics/robot-operating-system?o=desc&s=updated).

---

**Recommendations for getting started:**
- Fork and experiment with the C++ demos in `ros2/demos` and study implementations in the main `ros2/ros2` repo.
- Leverage packages such as `robot_state_publisher` and URDF tools for skeleton/joint management—key for humanoid robots.
- Explore the Poppy Project and similar open-source humanoid frameworks for direct examples in C++ and ROS integration.
- Keep an eye on curated lists like `awesome-ros2` for new humanoid packages and best practices.

These repositories form the backbone of modern humanoid robotics development with ROS. The flexibility of C++ for performance-critical code and the rich ecosystem of ROS packages make it straightforward to prototype, simulate, and deploy complex humanoid behaviors[[1]](https://roboticsbiz.com/ros-2-essential-resources-and-repositories-for-developers/)[[4]](https://github.com/ros)[[5]](https://thinkrobotics.com/blogs/learn/top-7-open-source-robotics-platforms)[[3]](https://github.com/ros2)[[6]](https://github.com/fkromer/awesome-ros2).

---
1. [ROS 2: Essential resources and repositories for developers](https://roboticsbiz.com/ros-2-essential-resources-and-repositories-for-developers/)
2. [GitHub - ros2/ros2: The Robot Operating System, is a meta operating ...](https://github.com/ros2/ros2)
3. [ROS 2 · GitHub](https://github.com/ros2)
4. [ROS core stacks - GitHub](https://github.com/ros)
5. [Top 7 Open-Source Robotics Platforms – ThinkRobotics.com](https://thinkrobotics.com/blogs/learn/top-7-open-source-robotics-platforms)
6. [Awesome Robot Operating System 2 (ROS 2) - GitHub](https://github.com/fkromer/awesome-ros2)
7. [robot-operating-system · GitHub Topics · GitHub](https://github.com/topics/robot-operating-system)
8. [robot-operating-system · GitHub Topics · GitHub](https://github.com/topics/robot-operating-system?o=desc&s=updated)


# (Rust/ROS 2 + Hardware)

---

## Table of Contents

* [1. Core Runtime & Communications (Rust)](#1-core-runtime--communications-rust)
* [2. Serialization & Wire Formats](#2-serialization--wire-formats)
* [3. Memory, Queues & Allocators](#3-memory-queues--allocators)
* [4. Storage & Columnar Data](#4-storage--columnar-data)
* [5. Observability & Telemetry](#5-observability--telemetry)
* [6. Error Correction & Robustness](#6-error-correction--robustness)
* [7. AI Inference (On‑device)](#7-ai-inference-ondevice)
* [8. ECS, Graphs & Spatial Indexing](#8-ecs-graphs--spatial-indexing)
* [9. Real‑Time OS & ROS 2 Integration](#9-realtime-os--ros-2-integration)
* [10. Simulation](#10-simulation)
* [11. Flight/Space Software Frameworks (Reference)](#11-flightspace-software-frameworks-reference)
* [12. Chipsets & Components (Docs/References)](#12-chipsets--components-docsreferences)

---

## 1. Core Runtime & Communications (Rust)

* **Tokio** — async runtime and I/O: [https://tokio.rs/](https://tokio.rs/)
* **Quinn (QUIC)** — QUIC transport in Rust: [https://github.com/quinn-rs/quinn](https://github.com/quinn-rs/quinn)
  QUIC spec (reference): [https://www.rfc-editor.org/rfc/rfc9000](https://www.rfc-editor.org/rfc/rfc9000)
* **Zenoh** — pub/sub + query + storage over unreliable links: [https://docs.zenoh.io/](https://docs.zenoh.io/)
* **RustDDS** — Data Distribution Service in Rust: [https://github.com/jhelovuo/RustDDS](https://github.com/jhelovuo/RustDDS)
* **rclrs (ROS 2 Rust client)** — ROS 2 client library for Rust: [https://github.com/ros2-rust/ros2_rust](https://github.com/ros2-rust/ros2_rust)
* **Cyclone DDS** (RMW) docs: [https://cyclonedds.io/docs/](https://cyclonedds.io/docs/)
* **Fast DDS** (RMW) docs: [https://fast-dds.docs.eprosima.com/](https://fast-dds.docs.eprosima.com/)
* **Delay‑Tolerant Networking (DTN)** — Bundle Protocol v7: [https://www.rfc-editor.org/rfc/rfc9171](https://www.rfc-editor.org/rfc/rfc9171)
* **NASA ION DTN** (reference implementation): [https://www.nasa.gov/general/interplanetary-internet/](https://www.nasa.gov/general/interplanetary-internet/)

## 2. Serialization & Wire Formats

* **Protocol Buffers (prost)**: [https://docs.rs/prost/latest/prost/](https://docs.rs/prost/latest/prost/)
* **Cap’n Proto (capnp)**: [https://capnproto.org/rust.html](https://capnproto.org/rust.html)
* **FlatBuffers (flatbuffers‑rust)**: [https://google.github.io/flatbuffers/](https://google.github.io/flatbuffers/)
  Rust crate index: [https://crates.io/crates/flatbuffers](https://crates.io/crates/flatbuffers)
* **Serde** ecosystem: [https://serde.rs/](https://serde.rs/)
  Binary: **bincode** [https://docs.rs/bincode/latest/bincode/](https://docs.rs/bincode/latest/bincode/)
* **rkyv** (zero‑copy): [https://docs.rs/rkyv/latest/rkyv/](https://docs.rs/rkyv/latest/rkyv/)

## 3. Memory, Queues & Allocators

* **bytes** (zero‑copy buffers): [https://docs.rs/bytes/latest/bytes/](https://docs.rs/bytes/latest/bytes/)
* **heapless** (no‑alloc DS for embedded): [https://docs.rs/heapless/latest/heapless/](https://docs.rs/heapless/latest/heapless/)
* **bbqueue** (SPSC lock‑free ring buffer): [https://docs.rs/bbqueue/latest/bbqueue/](https://docs.rs/bbqueue/latest/bbqueue/)
* **crossbeam** (lock‑free queues/tools): [https://crossbeam-rs.github.io/](https://crossbeam-rs.github.io/)
* **smallvec** (inline small vectors): [https://docs.rs/smallvec/latest/smallvec/](https://docs.rs/smallvec/latest/smallvec/)
* **arrayvec** (fixed‑cap vectors): [https://docs.rs/arrayvec/latest/arrayvec/](https://docs.rs/arrayvec/latest/arrayvec/)
* **bumpalo** (bump allocator): [https://docs.rs/bumpalo/latest/bumpalo/](https://docs.rs/bumpalo/latest/bumpalo/)
* **slotmap** (stable slot indices): [https://docs.rs/slotmap/latest/slotmap/](https://docs.rs/slotmap/latest/slotmap/)
* **generational‑arena**: [https://docs.rs/generational-arena/latest/generational_arena/](https://docs.rs/generational-arena/latest/generational_arena/)

## 4. Storage & Columnar Data

* **sled** (embedded DB): [https://sled.rs/](https://sled.rs/)
* **redb** (embedded DB): [https://docs.rs/redb/latest/redb/](https://docs.rs/redb/latest/redb/)
* **arrow2** (Apache Arrow in Rust): [https://docs.rs/arrow2/latest/arrow2/](https://docs.rs/arrow2/latest/arrow2/)
* **parquet2** (Parquet in Rust): [https://docs.rs/parquet2/latest/parquet2/](https://docs.rs/parquet2/latest/parquet2/)

## 5. Observability & Telemetry

* **tracing** (structured logging + spans): [https://docs.rs/tracing/latest/tracing/](https://docs.rs/tracing/latest/tracing/)
* **OpenTelemetry Rust**: [https://opentelemetry.io/docs/languages/rust/](https://opentelemetry.io/docs/languages/rust/)

## 6. Error Correction & Robustness

* **reed‑solomon‑erasure** (erasure coding): [https://docs.rs/reed-solomon-erasure/latest/reed_solomon_erasure/](https://docs.rs/reed-solomon-erasure/latest/reed_solomon_erasure/)
* **Linux EDAC** (ECC error reporting): [https://www.kernel.org/doc/html/latest/admin-guide/ras.html](https://www.kernel.org/doc/html/latest/admin-guide/ras.html) (general)
  Utilities: [https://github.com/grondo/edac-utils](https://github.com/grondo/edac-utils)
* **SECDED/Chipkill** (background): Micron ECC intro [https://www.micron.com/about/blog/memory/dram/advantage-of-error-correcting-code-ecc-dram-in-smartphones](https://www.micron.com/about/blog/memory/dram/advantage-of-error-correcting-code-ecc-dram-in-smartphones) ; NASA RS tutorial [https://ntrs.nasa.gov/api/citations/19900019023/downloads/19900019023.pdf](https://ntrs.nasa.gov/api/citations/19900019023/downloads/19900019023.pdf)

## 7. AI Inference (On‑device)

* **ONNX Runtime (Rust bindings)**: [https://github.com/microsoft/onnxruntime-rs](https://github.com/microsoft/onnxruntime-rs)
* **tch‑rs (LibTorch)**: [https://github.com/LaurentMazare/tch-rs](https://github.com/LaurentMazare/tch-rs)
* **candle** (pure‑Rust ML): [https://huggingface.co/docs/candle](https://huggingface.co/docs/candle)
* **safetensors**: [https://github.com/huggingface/safetensors](https://github.com/huggingface/safetensors)

## 8. ECS, Graphs & Spatial Indexing

* **bevy_ecs**: [https://docs.rs/bevy_ecs/latest/bevy_ecs/](https://docs.rs/bevy_ecs/latest/bevy_ecs/)
* **hecs**: [https://docs.rs/hecs/latest/hecs/](https://docs.rs/hecs/latest/hecs/)
* **petgraph** (graphs): [https://petgraph.org/](https://petgraph.org/)
* **kiddo** (k‑d tree): [https://docs.rs/kiddo/latest/kiddo/](https://docs.rs/kiddo/latest/kiddo/)

## 9. Real‑Time OS & ROS 2 Integration

* **PREEMPT_RT** (Linux RT): [https://wiki.linuxfoundation.org/realtime/documentation/start](https://wiki.linuxfoundation.org/realtime/documentation/start)
  Kernel docs: [https://www.kernel.org/doc/html/next/core-api/real-time/index.html](https://www.kernel.org/doc/html/next/core-api/real-time/index.html)
* **Memory Locking** — `mlockall(2)`: [https://man7.org/linux/man-pages/man2/mlock.2.html](https://man7.org/linux/man-pages/man2/mlock.2.html)
* **micro‑ROS** (RTOS MCUs): [https://micro.ros.org/docs/overview/](https://micro.ros.org/docs/overview/)
* **ros2_control** (hardware abstraction/RT control loop): [https://control.ros.org/](https://control.ros.org/)
* **colcon** (ROS 2 build tool): [https://colcon.readthedocs.io/](https://colcon.readthedocs.io/)

## 10. Simulation

* **Gazebo (gz)**: [https://gazebosim.org/docs](https://gazebosim.org/docs)
* **NVIDIA Isaac Sim/Lab**: [https://docs.nvidia.com/isaac/](https://docs.nvidia.com/isaac/) (high‑fidelity GPU sim)

## 11. Flight/Space Software Frameworks (Reference)

* **NASA core Flight System (cFS)**: [https://github.com/nasa/cFS](https://github.com/nasa/cFS) • [https://etd.gsfc.nasa.gov/capabilities/core-flight-system/](https://etd.gsfc.nasa.gov/capabilities/core-flight-system/)
* **NASA/JPL F´ (F Prime)**: [https://nasa.github.io/fprime/](https://nasa.github.io/fprime/) • [https://github.com/nasa/fprime](https://github.com/nasa/fprime)

---

## 12. Chipsets & Components (Docs/References)

### Edge/Embedded Compute SoCs & Modules

* **NVIDIA Jetson Orin (AGX/NX/Nano)**
  User guide & downloads: [https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/1.0/index.html](https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/1.0/index.html) • [https://developer.nvidia.com/embedded/downloads](https://developer.nvidia.com/embedded/downloads)
* **NXP i.MX 8M Plus** (A53 + M7 + NPU)
  Reference manual: [https://www.nxp.com/docs/en/preview/PREVIEW_IMX8MPRM.pdf](https://www.nxp.com/docs/en/preview/PREVIEW_IMX8MPRM.pdf) (Rev history: [https://www.nxp.com/pcn/202402013I](https://www.nxp.com/pcn/202402013I))
* **TI Sitara AM64x / AM67A**
  AM64x TRM/SDK: [https://www.ti.com/processors/sitara-arm/am64x-cortex-a53-m4/products.html](https://www.ti.com/processors/sitara-arm/am64x-cortex-a53-m4/products.html)
  AM67A (with C7x DSP): [https://www.ti.com/product/AM67A](https://www.ti.com/product/AM67A)
* **STM32H7** (High‑perf MCU)
  Reference manuals/datasheets: [https://www.st.com/en/microcontrollers-microprocessors/stm32h7-series.html](https://www.st.com/en/microcontrollers-microprocessors/stm32h7-series.html)
* **ESP32‑S3** (Dual‑core MCU, AI accel)
  TRM & docs: [https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
* **Microchip PolarFire® SoC** (RISC‑V + FPGA)
  Docs hub: [https://www.microchip.com/en-us/products/fpgas-and-plds/soc-fpgas/polarfire-soc-fpga](https://www.microchip.com/en-us/products/fpgas-and-plds/soc-fpgas/polarfire-soc-fpga)
* **RT PolarFire® (Space‑grade FPGA)**
  Product/docs: [https://www.microchip.com/en-us/products/fpgas-and-plds/space-and-defense-fpgas/rt-polarfire-fpga](https://www.microchip.com/en-us/products/fpgas-and-plds/space-and-defense-fpgas/rt-polarfire-fpga)
* **AMD/Xilinx Space‑grade Versal (XQR)**
  Overview/docs: [https://www.amd.com/en/products/adaptive-socs-and-fpgas/space.html](https://www.amd.com/en/products/adaptive-socs-and-fpgas/space.html)
* **Frontgrade Gaisler GR740 (LEON4FT)**
  Datasheet & guides: [https://www.gaisler.com/index.php/products/space-gr740](https://www.gaisler.com/index.php/products/space-gr740)
* **AMD Ryzen Embedded (V‑/R‑Series)**
  Family overview/docs: [https://www.amd.com/en/products/embedded](https://www.amd.com/en/products/embedded)

### AI/ML Accelerators

* **NVIDIA Jetson (TensorRT on Orin)** — see Jetson docs above
* **Google Coral Edge TPU**
  Dev Board datasheet: [https://www.coral.ai/docs/dev-board/datasheet/](https://www.coral.ai/docs/dev-board/datasheet/) • SoM datasheet: [https://coral.ai/static/files/Coral-SoM-datasheet.pdf](https://coral.ai/static/files/Coral-SoM-datasheet.pdf)
* **Hailo‑8**
  Product brief & datasheets: [https://hailo.ai/products/ai-accelerators/hailo-8-ai-accelerator/](https://hailo.ai/products/ai-accelerators/hailo-8-ai-accelerator/)
* **Intel Movidius Myriad X (legacy)**
  OpenVINO notes & HAL guide: [https://docs.openvino.ai/2024/about-openvino/compatibility-and-support/supported-devices.html](https://docs.openvino.ai/2024/about-openvino/compatibility-and-support/supported-devices.html) • [https://docs.openvino.ai/downloads/595850_Intel_Vision_Accelerator_Design_with_Intel_Movidius%E2%84%A2_VPUs-HAL%20Configuration%20Guide_rev1.3.pdf](https://docs.openvino.ai/downloads/595850_Intel_Vision_Accelerator_Design_with_Intel_Movidius%E2%84%A2_VPUs-HAL%20Configuration%20Guide_rev1.3.pdf)

### Comms Buses & Timing (Reference)

* **QUIC** (transport): [https://www.rfc-editor.org/rfc/rfc9000](https://www.rfc-editor.org/rfc/rfc9000)
* **DTN Bundle Protocol v7**: [https://www.rfc-editor.org/rfc/rfc9171](https://www.rfc-editor.org/rfc/rfc9171)
* **SpaceWire** (overview): [https://www.star-dundee.com/knowledge/spacewire/](https://www.star-dundee.com/knowledge/spacewire/)
* **SpaceFibre** (overview): [https://www.esa.int/Enabling_Support/Space_Engineering_Technology/SpaceFibre](https://www.esa.int/Enabling_Support/Space_Engineering_Technology/SpaceFibre)
* **IEEE 1588 PTP** (timing) — linuxptp project docs: [https://linuxptp.sourceforge.net/](https://linuxptp.sourceforge.net/)

### OS/Kernel Reliability

* **PREEMPT_RT**: [https://wiki.linuxfoundation.org/realtime/documentation/start](https://wiki.linuxfoundation.org/realtime/documentation/start) • [https://www.kernel.org/doc/html/next/core-api/real-time/index.html](https://www.kernel.org/doc/html/next/core-api/real-time/index.html)
* **Memory Locking**: `mlockall(2)` man page — [https://man7.org/linux/man-pages/man2/mlock.2.html](https://man7.org/linux/man-pages/man2/mlock.2.html)
* **Linux EDAC**: [https://www.kernel.org/doc/html/latest/admin-guide/ras.html](https://www.kernel.org/doc/html/latest/admin-guide/ras.html) • [https://github.com/grondo/edac-utils](https://github.com/grondo/edac-utils)

---

### Notes

* Prefer **official vendor documentation** for pinouts, power budgets, and errata.
* For spaceflight heritage and radiation data, see vendor “space/defense” product pages (Microchip RT, AMD/Xilinx Space, Gaisler).
* Keep this doc versioned in the repo and update links with specific revision numbers when we lock designs.


---


## 1. Modern C++ Fundamentals

### Core C++17/20 Features for Robotics

```cpp
// Smart pointers for automatic memory management
#include <memory>
std::unique_ptr<RobotController> controller = std::make_unique<RobotController>();
std::shared_ptr<SensorData> data = std::make_shared<SensorData>();

// Move semantics for performance
class LidarScan {
public:
    LidarScan(std::vector<float>&& ranges) 
        : ranges_(std::move(ranges)) {}  // Efficient move
    
    // Rule of Five
    LidarScan(const LidarScan&) = delete;
    LidarScan& operator=(const LidarScan&) = delete;
    LidarScan(LidarScan&&) = default;
    LidarScan& operator=(LidarScan&&) = default;
    ~LidarScan() = default;

private:
    std::vector<float> ranges_;
};

// Structured bindings
auto [position, orientation] = getPose();

// constexpr for compile-time computation
constexpr double PI = 3.141592653589793;
constexpr double degToRad(double deg) { return deg * PI / 180.0; }
```

### Essential Standard Library Usage

```cpp
#include <vector>
#include <array>
#include <algorithm>
#include <functional>

// Use std::array for fixed-size data (common in robotics)
std::array<double, 3> position = {0.0, 0.0, 0.0};

// Algorithms for sensor data processing
std::vector<double> sensor_readings = {1.1, 2.2, 3.3, 4.4};
std::sort(sensor_readings.begin(), sensor_readings.end());
auto it = std::find_if(sensor_readings.begin(), sensor_readings.end(),
                      [](double val) { return val > 3.0; });

// Lambdas for callbacks
auto process_sensor_data = [](const auto& data) {
    // Process data
};
```

---

## 2. Robotics-Specific C++ Patterns

### Real-Time Safe Patterns

```cpp
// Pre-allocate memory to avoid dynamic allocation in real-time threads
class RealTimeController {
private:
    std::array<double, 1000> command_buffer_;  // Pre-allocated
    size_t buffer_index_ = 0;
    
public:
    void addCommand(double command) {
        if (buffer_index_ < command_buffer_.size()) {
            command_buffer_[buffer_index_++] = command;
        }
    }
};

// Ring buffer for sensor data
template<typename T, size_t N>
class SensorRingBuffer {
private:
    std::array<T, N> buffer_;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;

public:
    bool push(const T& item) {
        if (count_ >= N) return false;
        buffer_[head_] = item;
        head_ = (head_ + 1) % N;
        ++count_;
        return true;
    }
    
    bool pop(T& item) {
        if (count_ == 0) return false;
        item = buffer_[tail_];
        tail_ = (tail_ + 1) % N;
        --count_;
        return true;
    }
};
```

### Thread-Safe Data Sharing

```cpp
#include <mutex>
#include <atomic>

class ThreadSafeSensorData {
private:
    mutable std::mutex mutex_;
    std::vector<double> data_;
    std::atomic<bool> new_data_available_{false};

public:
    void updateData(const std::vector<double>& new_data) {
        std::lock_guard<std::mutex> lock(mutex_);
        data_ = new_data;
        new_data_available_.store(true, std::memory_order_release);
    }
    
    std::vector<double> getData() const {
        std::lock_guard<std::mutex> lock(mutex_);
        new_data_available_.store(false, std::memory_order_release);
        return data_;
    }
    
    bool isNewDataAvailable() const {
        return new_data_available_.load(std::memory_order_acquire);
    }
};
```

### Configuration Management

```cpp
#include <unordered_map>
#include <string>
#include <variant>

using ConfigValue = std::variant<int, double, bool, std::string>;

class RobotConfig {
private:
    std::unordered_map<std::string, ConfigValue> config_;

public:
    template<typename T>
    void set(const std::string& key, const T& value) {
        config_[key] = value;
    }
    
    template<typename T>
    T get(const std::string& key, const T& default_value) const {
        auto it = config_.find(key);
        if (it != config_.end()) {
            try {
                return std::get<T>(it->second);
            } catch (const std::bad_variant_access&) {
                return default_value;
            }
        }
        return default_value;
    }
};
```

---

## 3. Essential Libraries & Tools

### CMake for Robotics Projects

```cmake
# Minimum CMake version
cmake_minimum_required(VERSION 3.16)
project(robotics_controller VERSION 1.0.0 LANGUAGES CXX)

# Modern C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Essential flags for robotics
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -O2")

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# Add your targets
add_library(robot_core 
    src/controller.cpp
    src/navigation.cpp
    src/perception.cpp
)

# Link dependencies
target_link_libraries(robot_core
    PUBLIC
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
)

# Executable
add_executable(robot_main src/main.cpp)
target_link_libraries(robot_main robot_core)
```

### Essential Third-Party Libraries

```cpp
// Eigen for linear algebra
#include <Eigen/Dense>
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Quaterniond = Eigen::Quaterniond;

class Pose {
private:
    Vector3d position_;
    Quaterniond orientation_;

public:
    Pose transform(const Pose& other) const {
        Pose result;
        result.position_ = position_ + orientation_ * other.position_;
        result.orientation_ = orientation_ * other.orientation_;
        return result;
    }
};

// JSON for configuration (nlohmann/json)
#include <nlohmann/json.hpp>
using json = nlohmann::json;

json config = json::parse(R"({
    "max_velocity": 2.0,
    "sensor_rate": 100,
    "controller_gains": [1.0, 0.5, 0.1]
})");

double max_vel = config["max_velocity"];
```

---

## 4. ROS 2 Integration

### Basic ROS 2 Node Structure

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class RobotController : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

public:
    RobotController() : Node("robot_controller") {
        // Parameters
        this->declare_parameter<double>("max_speed", 1.0);
        this->declare_parameter<double>("safety_distance", 0.5);
        
        // Subscribers
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&RobotController::lidarCallback, this, std::placeholders::_1));
            
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Control timer (100Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&RobotController::controlLoop, this));
            
        RCLCPP_INFO(this->get_logger(), "Robot controller initialized");
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Process lidar data
        double min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        
        double safety_dist = this->get_parameter("safety_distance").as_double();
        if (min_distance < safety_dist) {
            emergencyStop();
        }
    }
    
    void controlLoop() {
        auto command = geometry_msgs::msg::Twist();
        // Compute control command
        cmd_vel_pub_->publish(command);
    }
    
    void emergencyStop() {
        auto stop_cmd = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_WARN(this->get_logger(), "Emergency stop triggered!");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Custom Message Handling

```cpp
#include <memory>
#include <utility>

template<typename MsgType>
class MessageProcessor {
public:
    virtual void process(const MsgType& msg) = 0;
    virtual ~MessageProcessor() = default;
};

class OdometryProcessor : public MessageProcessor<nav_msgs::msg::Odometry> {
public:
    void process(const nav_msgs::msg::Odometry& odom) override {
        // Process odometry data
        current_pose_ = odom.pose.pose;
        velocity_ = odom.twist.twist;
        updateStateEstimation();
    }
    
private:
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist velocity_;
    
    void updateStateEstimation() {
        // Kalman filter update, etc.
    }
};
```

---

## 5. Project Structure & Best Practices

### Recommended Project Layout

```
robotics_project/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── project_name/
│       ├── controller.hpp
│       ├── navigation.hpp
│       └── perception.hpp
├── src/
│   ├── controller.cpp
│   ├── navigation.cpp
│   ├── perception.cpp
│   └── main.cpp
├── launch/
│   └── robot.launch.py
├── config/
│   └── params.yaml
├── test/
│   ├── test_controller.cpp
│   └── test_navigation.cpp
└── scripts/
    └── calibration.py
```

### Testing Framework

```cpp
#include <gtest/gtest.h>
#include "controller.hpp"

class ControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller_ = std::make_unique<RobotController>();
    }
    
    void TearDown() override {
        controller_.reset();
    }
    
    std::unique_ptr<RobotController> controller_;
};

TEST_F(ControllerTest, EmergencyStop) {
    std::vector<double> close_obstacles = {0.1, 0.2, 0.3};
    auto command = controller_->computeCommand(close_obstacles);
    
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST_F(ControllerTest, NormalOperation) {
    std::vector<double> clear_path = {1.0, 2.0, 3.0};
    auto command = controller_->computeCommand(clear_path);
    
    EXPECT_GT(command.linear.x, 0.0);
}
```

### Logging and Debugging

```cpp
#include <iostream>
#include <chrono>
#include <iomanip>

class ScopedTimer {
public:
    ScopedTimer(const std::string& name) 
        : name_(name), start_(std::chrono::high_resolution_clock::now()) {}
        
    ~ScopedTimer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_);
        std::cout << name_ << " took " << duration.count() << " μs\n";
    }
    
private:
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

// Usage
void processSensorData() {
    ScopedTimer timer("Sensor Processing");
    // ... processing code
}
```

---

## 6. Learning Path

### Phase 1: Foundation (1-2 months)
- Modern C++ syntax (smart pointers, lambdas, move semantics)
- CMake basics
- Standard Library containers and algorithms
- Basic multithreading

### Phase 2: Robotics Core (2-3 months)
- ROS 2 node development
- Eigen for math operations
- Sensor data handling patterns
- Basic control algorithms

### Phase 3: Advanced Patterns (3-4 months)
- Real-time programming concepts
- Performance optimization
- System integration patterns
- Testing and validation

### Phase 4: Specialization (Ongoing)
- Perception pipelines (OpenCV, PCL)
- Motion planning algorithms
- ML integration (PyTorch C++ API)
- Safety-critical development

### Essential Resources
- **Books**: "Effective Modern C++", "ROS 2 Documentation"
- **Libraries**: Eigen, PCL, OpenCV, nlohmann/json
- **Tools**: GCC/Clang, GDB, Valgrind, Clang-Tidy
- **Platforms**: Ubuntu 22.04, ROS 2 Humble


Here are authoritative URLs for the recommended books, courses, libraries, simulation platforms, open-source projects, and professional communities featured throughout the Ultimate Humanoid Robotics Resource Guide. Each resource is matched directly with its website or hosting platform for timely access and reference.

***

### 📚 Foundational Knowledge

**Mathematics for Robotics**
- “Mathematics for Robotics” by Peter Corke (free online chapters):  
  https://petercorke.com/toolboxes/robotics-toolbox/
- MIT 18.06 Linear Algebra (OCW):  
  https://ocw.mit.edu/courses/mathematics/18-06-linear-algebra-spring-2010/
- Khan Academy Linear Algebra & Calculus:  
  https://www.khanacademy.org/math/linear-algebra  
  https://www.khanacademy.org/math/calculus-1
- "Probability and Statistics for Engineering" by Devore:  
  https://www.cengage.com/c/probability-and-statistics-for-engineering-and-the-sciences-9e-devore/9781305251809PF/

**Physics & Mechanics**
- “Classical Mechanics” by John Taylor:  
  https://global.oup.com/ushe/product/classical-mechanics-9781891389221
- MIT 8.01 Physics I (OCW):  
  https://ocw.mit.edu/courses/physics/8-01sc-classical-mechanics-fall-2016/
- PhET Interactive Simulations:  
  https://phet.colorado.edu/

***

### 🤖 Core Robotics Fundamentals

**ROS 2 & Simulation**
- ROS 2 Official Tutorials:  
  https://docs.ros.org/en/rolling/index.html
- The Construct ROS 2 Courses:  
  https://www.theconstruct.ai/
- “Mastering ROS 2” by Lentin Joseph:  
  https://www.packtpub.com/product/mastering-ros-for-robotics-programming-second-edition/9781788623396
- ROS 2 Awesome List (GitHub):  
  https://github.com/fkromer/awesome-ros2

**Gazebo/Ignition Simulation**
- Gazebo Official Simulator:  
  https://gazebosim.org/
- Webots:  
  https://cyberbotics.com/
- MuJoCo:  
  https://mujoco.org/
- NVIDIA Isaac Sim:  
  https://developer.nvidia.com/isaac-sim

***

### Programming Foundations

**C++ for Robotics**
- “Effective Modern C++” by Scott Meyers:  
  https://www.oreilly.com/library/view/effective-modern-c/9781491908419/
- C++ for Robotics (The Construct):  
  https://app.theconstruct.ai/courses/59
- LeetCode for algorithm practice:  
  https://leetcode.com/

**Python for Robotics**
- “Python for Robotics” (The Construct):  
  https://app.theconstruct.ai/courses/58
- NumPy:  
  https://numpy.org/
- SciPy:  
  https://scipy.org/
- Matplotlib:  
  https://matplotlib.org/
- OpenCV:  
  https://opencv.org/

***

### 🦾 Humanoid-Specific Knowledge

**Kinematics & Dynamics**
- “Rigid Body Dynamics” by Roy Featherstone:  
  https://www.springer.com/gp/book/9780387743125
- Stanford CS223A - Introduction to Robotics:  
  https://web.stanford.edu/class/cs223a/
- Khatib whole-body control paper:  
  https://ieeexplore.ieee.org/document/390988
- PRATT balance control paper:  
  https://ieeexplore.ieee.org/document/865360
- Drake (MIT robot dynamics toolkit):  
  https://drake.mit.edu/

**Control Systems**
- “Robot Dynamics and Control” by Spong et al.:  
  https://www.wiley.com/en-us/Robot+Modeling+and+Control%2C+2nd+Edition-p-9780471649908
- MIT Underactuated Robotics (Russ Tedrake, OCW):  
  https://underactuated.mit.edu/
- OCS2 (Optimal Control Software):  
  https://github.com/leggedrobotics/ocs2
- MATLAB Robotics System Toolbox:  
  https://www.mathworks.com/products/robotics.html

***

### 🔧 Hardware & Electronics

**Motors/Actuators**
- Maxon Motors (technical docs):  
  https://www.maxongroup.com/maxon/view/content/technical-documentation
- Harmonic Drive:  
  https://harmonicdrive.net/
- “Electric Motors and Drives” by Austin Hughes:  
  https://www.elsevier.com/books/electric-motors-and-drives/hughes/978-0-08-098332-5

**Sensors**
- Bosch Sensortec:  
  https://www.bosch-sensortec.com/
- TDK InvenSense:  
  https://www.invensense.tdk.com/
- “Applied Inertial Navigation” by Groves:  
  https://www.springer.com/gp/book/9783030239561
- ROS Sensor Drivers:  
  https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/#install-ros-2-packages

**Power Management**
- Battery University:  
  https://batteryuniversity.com/
- IEC 62133 standard summary:  
  https://webstore.iec.ch/publication/27200
- LTspice simulation:  
  https://www.analog.com/en/design-center/design-tools-and-calculators/ltspice-simulator.html

***

### 🧠 AI & Advanced Algorithms

**Computer Vision**
- CS231n Convolutional Neural Networks (Stanford):  
  http://cs231n.stanford.edu/
- “Computer Vision: Algorithms and Applications” by Szeliski:  
  http://szeliski.org/Book/
- Detectron2 (Meta AI):  
  https://github.com/facebookresearch/detectron2

**Machine Learning**
- CS229 Machine Learning (Stanford):  
  https://cs229.stanford.edu/
- PyTorch:  
  https://pytorch.org/
- TensorFlow:  
  https://www.tensorflow.org/
- Stable Baselines3:  
  https://stable-baselines3.readthedocs.io/
- OpenAI Gym:  
  https://www.gymlibrary.dev/
- DeepMind locomotion papers:  
  https://deepmind.com/research/publications

**Behavior Trees**
- BehaviorTree.CPP:  
  https://github.com/BehaviorTree/BehaviorTree.CPP
- “Behavior Trees in Robotics” by Colledanchise:  
  https://www.springer.com/gp/book/9783030056397
- ROS 2 Behavior Tree Tutorial:  
  https://navigation.ros.org/tutorials/docs/navigation2_with_behavior_trees.html

***

### 🛠️ Development Tools & Platforms

**Simulation**
- CoppeliaSim (V-REP):  
  https://www.coppeliarobotics.com/
- RoboDK:  
  https://robodk.com/

**CAD & Mechanical**
- SolidWorks:  
  https://www.solidworks.com/
- Fusion 360:  
  https://www.autodesk.com/products/fusion-360/
- FreeCAD:  
  https://www.freecadweb.org/
- Blender:  
  https://www.blender.org/
- GrabCAD:  
  https://grabcad.com/
- Onshape:  
  https://www.onshape.com/
- McMaster-Carr CAD Library:  
  https://www.mcmaster.com/cad-library/

**Electronics Design**
- KiCad:  
  https://www.kicad.org/
- LTspice:  
  https://www.analog.com/en/design-center/design-tools-and-calculators/ltspice-simulator.html
- EAGLE:  
  https://www.autodesk.com/products/eagle/

***

### 📋 Projects & Open Source

**Poppy Project (Open humanoid platform):**  
https://www.poppy-project.org/en/

**Open Dynamic Robot Initiative:**  
https://open-dynamic-robot-initiative.github.io/

**Stanford Doggo:**  
https://github.com/stanfordroboticsclub/doggo

**Unitree Robotics SDK (Unitree G1):**  
https://www.unitree.com/download/

***

### 🏛️ Research, Community, and Conferences

**Stanford Robotics Lab:**  
https://robotics.stanford.edu/

**MIT CSAIL:**  
https://www.csail.mit.edu/research/robotics

**CMU Robotics Institute:**  
https://www.ri.cmu.edu/

**University of Tokyo JSK Lab:**  
https://www.jsk.t.u-tokyo.ac.jp/index_en.html

**IIT Humanoid & Human Centered Mechatronics Lab:**  
https://www.iit.it/research/centers/robotics/humanoid-and-human-centered-mechatronics

**IEEE-RAS Humanoids Conference:**  
https://www.humanoids2023.org/  
**ICRA:**  
https://www.ieee-icra.org/  
**IROS:**  
https://www.iros2023.org/  
**RSS:**  
https://roboticsconference.org/

***

### 📈 Career & Job Search

**ROS Jobs:**  
https://jobs.ros.org/

**IEEE RAS Job Board:**  
https://www.ieee-ras.org/ras-job-board

**Boston Dynamics Careers:**  
https://www.bostondynamics.com/careers/

**Tesla Robotics Careers:**  
https://www.tesla.com/careers/search/?category=robotics

**Figure AI Careers:**  
https://www.figure.ai/careers

**Unitree Robotics Careers:**  
https://www.unitree.com/aboutus/hr/

***

### 🆘 Troubleshooting & Support

**ROS Answers:**  
https://answers.ros.org/

**Robotics Stack Exchange:**  
https://robotics.stackexchange.com/

**PlotJuggler:**  
https://www.plotjuggler.io/

**Wireshark:**  
https://www.wireshark.org/

***
