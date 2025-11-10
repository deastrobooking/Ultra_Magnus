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

# Ultra Magnus — Space Robotics Reference Guide (Rust/ROS 2 + Hardware)

A living catalog of the frameworks, crates, standards, and hardware we’ll use to build a space‑hardened, ROS 2‑compatible robotics stack in Rust.

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
# Rust for Robotics
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

* Core runtime/comms (Tokio; QUIC via Quinn; Zenoh & RustDDS; DTN BPv7) and where each fits, with official specs and docs.
* ROS 2 integration (ros2_control, colcon) plus micro-ROS for MCU-class nodes.
* Serialization stacks (prost/Cap’n Proto/FlatBuffers), memory/queue crates (heapless, bbqueue, smallvec, bytes, bumpalo, slotmap, generational-arena), storage (sled/redb; Arrow/Parquet), observability (tracing, OpenTelemetry). ([Docs.rs][1])
* Fault-tolerance bits (Reed–Solomon; EDAC pointers), real-time Linux notes (PREEMPT_RT + `mlockall`).
* On-device AI (ONNX Runtime Rust, tch-rs/LibTorch, Candle, safetensors) and data formats you’ll likely use in space-constrained pipelines.
* Hardware shortlist with direct docs: Jetson Orin, i.MX 8M Plus, TI AM64x/AM67A, STM32H7, ESP32-S3, Microchip PolarFire/RT PolarFire, AMD/Xilinx space-grade Versal, Gaisler GR740, plus accelerators (Hailo-8, Coral Edge TPU, Movidius notes).
[1]: https://docs.rs/arrow2 "arrow2 - Rust"



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

This guide provides a solid foundation for starting C++ robotics development while emphasizing modern practices that will serve you well in production environments.
