# Autonomous Object Detection Self-Balancing Robot

## Project Overview

This project implements a two-wheeled self-balancing robot with autonomous navigation and object detection capabilities. The robot combines classical control theory (PID for balancing), computer vision (YOLO object detection), and SLAM algorithms to achieve autonomous operation in real-world environments.

### Key Features

- **Self-Balancing**: Uses MPU6050 IMU with PID control for real-time balance maintenance
- **Visual Odometry**: ESP32-CAM (OV7670 sensor) provides visual feedback for navigation
- **Object Detection**: YOLOv8-based detection running on server or edge FPGA
- **Distance Mapping**: HC-SR04 ultrasonic sensor compensates for single-camera depth limitations
- **Dual Operation Modes**: 
  - Server Mode: Processing on PC with network commands
  - Edge Mode: On-board FPGA processing with minimal latency
- **Adjustable FOV**: MG90 servo controls camera pitch for expanded field of view

## System Architecture

The robot operates on a distributed architecture with three main processing units:

1. **ESP32 (Main MCU)**: Motor control, sensor fusion, balance control, network communication
2. **ESP32-CAM**: Video streaming and visual odometry
3. **FPGA (Co-processor)**: Edge computing for real-time object detection and SLAM

### Hardware Stack Organization

The robot is built on a three-layer stack:

- **Base Stack**: Motors with 3D-printed motor mounts, battery compartment
- **Second Stack**: Main circuitry, motor driver, buck converters
- **Third Stack**: ESP32-CAM, servo mechanism, FPGA module

## Operational Modes

### Server Mode
In this mode, the robot acts as a remote sensor platform:
- ESP32 streams sensor data (IMU, ultrasonic) via UDP
- ESP32-CAM streams video frames via TCP
- PC/laptop runs object detection (YOLOv8) and SLAM
- Control commands sent back to robot via UDP
- Suitable for development, testing, and high-accuracy processing

### Edge Mode
Autonomous operation with on-board intelligence:
- FPGA performs object detection and SLAM locally
- ESP32 maintains balance and executes navigation commands
- Minimal latency for real-time obstacle avoidance
- Robot streams telemetry to app/PC for monitoring only
- Suitable for deployment in environments without reliable network

## Communication Flow

```
Sensor Data Flow:
MPU6050 → ESP32 → UDP → Server/App
HC-SR04 → ESP32 → UDP → Server/App
OV7670 → ESP32-CAM → TCP → Server/App

Control Flow:
Server/App → UDP → ESP32 → Motor Driver → Motors
FPGA → ESP32 → Motor Driver → Motors (Edge Mode)

Video Flow:
ESP32-CAM → TCP Stream → Server/App
```

## Project Goals

1. Achieve stable self-balancing under various load conditions
2. Implement reliable object detection and classification
3. Create accurate distance mapping using sensor fusion
4. Enable autonomous navigation with obstacle avoidance
5. Provide flexible control via mobile app or autonomous algorithms
6. Demonstrate edge computing capabilities with FPGA integration
