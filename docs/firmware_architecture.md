# Firmware Architecture

## Overview

The firmware is divided into two main ESP32 programs, each handling specific responsibilities in the robot's operation. Both are built using ESP-IDF framework and organized as component-based architecture.

## Firmware Projects

### Control Firmware (`firmware/control/`)

The primary firmware running on the main ESP32, responsible for balance, navigation, and sensor management.

#### Core Components

##### IMU Component (`components/imu/`)
- **Purpose**: Interface with MPU6050 sensor
- **Functions**:
  - Sensor initialization and calibration
  - Gyroscope and accelerometer data reading
  - Complementary filter for pitch angle estimation
  - Temperature compensation
- **Key Files**:
  - `config.hpp`: I2C configuration, sensor addresses, filter constants
  - `imu.hpp/cpp`: MPU6050 driver implementation

##### Motor Driver Component (`components/motor_driver/`)
- **Purpose**: Control DRV8833 H-bridge motor driver
- **Functions**:
  - PWM generation for motor speed control
  - Direction control (forward/backward)
  - Differential drive calculations for turning
  - Safety limits and deadband management
- **Key Files**:
  - `motor_driver.hpp/cpp`: DRV8833 control interface

##### Servo Component (`components/servo/`)
- **Purpose**: Control MG90 servo for camera positioning
- **Functions**:
  - PWM generation for servo angle control
  - Smooth angle transitions
  - Position limits and safety checks
- **Key Files**:
  - `servo.hpp/cpp`: Servo motor control implementation

##### Ultrasound Sensor Component (`components/ultrasound_sensor/`)
- **Purpose**: Interface with HC-SR04 distance sensor
- **Functions**:
  - Trigger pulse generation
  - Echo time measurement
  - Distance calculation (time-of-flight)
  - Filtering for noise reduction
- **Key Files**:
  - `ultrasound_sensor.hpp/cpp`: HC-SR04 driver

#### Main Application (`main/`)

##### Gyro Context (`gyro_context.hpp/cpp`)
- **Purpose**: Central coordination and state management
- **Responsibilities**:
  - Task orchestration
  - Sensor data fusion
  - Control loop management
  - Network message handling

##### Main Entry Point (`main.cpp`)
- **Boot Sequence**:
  1. Initialize WiFi station (connect to network)
  2. Initialize UDP client for bidirectional communication
  3. Initialize hardware components (IMU, motors, servo, ultrasonic)
  4. Create FreeRTOS tasks
  5. Start main control loop

#### FreeRTOS Tasks

The firmware runs multiple concurrent tasks:

1. **Balance Control Task**
   - Priority: High (real-time critical)
   - Frequency: ~100-200 Hz
   - Function: PID control loop for self-balancing
   - Reads IMU pitch angle
   - Calculates motor corrections
   - Updates motor PWM

2. **Sensor Reading Task**
   - Priority: Medium-High
   - Function: Periodic sensor data acquisition
   - Reads ultrasonic distance
   - Reads IMU raw data
   - Publishes data to network via UDP

3. **Network Command Task**
   - Priority: Medium
   - Function: Receive and process UDP commands
   - Parses joystick input
   - Processes autonomous navigation commands
   - Updates servo position commands

4. **Telemetry Task**
   - Priority: Low
   - Frequency: ~10-20 Hz
   - Function: Send status updates via UDP
   - Battery voltage
   - Sensor readings
   - Motor states

### Vision Firmware (`firmware/vision/`)

Dedicated firmware for ESP32-CAM module, focused on image acquisition and streaming.

#### Camera Component (`components/camera/`)
- **Purpose**: Interface with OV7670 camera sensor
- **Functions**:
  - Camera initialization and configuration
  - Frame capture from camera buffer
  - JPEG compression (if supported)
  - Frame rate management
- **Key Files**:
  - `camera.hpp/cpp`: OV7670 driver implementation

#### Main Application (`main/`)

##### Vision Context (`vision_context.hpp/cpp`)
- **Purpose**: Manage camera and streaming operations
- **Responsibilities**:
  - Frame buffer management
  - TCP connection handling
  - Frame transmission synchronization

##### Main Entry Point (`main.cpp`)
- **Boot Sequence**:
  1. Initialize WiFi station
  2. Initialize TCP client
  3. Configure camera sensor
  4. Start streaming task

#### FreeRTOS Tasks

1. **Camera Capture Task**
   - Priority: High
   - Function: Continuous frame acquisition
   - Captures frames from OV7670
   - Places frames in buffer queue

2. **TCP Streaming Task**
   - Priority: Medium
   - Function: Send frames over network
   - Retrieves frames from buffer
   - Sends via TCP to server/app
   - Handles connection management

## Networking Components (`firmware/components/networking/`)

Shared networking library used by both firmware projects.

### WiFi Station (`wifi_station.hpp/cpp`)
- WiFi connection management
- Auto-reconnection logic
- SSID/password configuration
- Event handling (connected, disconnected)

### UDP Client (`udp_client.hpp/cpp`)
- Bidirectional UDP communication
- Packet transmission
- Packet reception with callbacks
- Non-blocking operations

### TCP Client (`tcp_client.hpp/cpp`)
- TCP connection management
- Stream-based data transmission
- Connection retry logic
- Keep-alive mechanisms

## Common Utilities (`firmware/common/`)

### Common Header (`common.hpp`)
- Shared constants and definitions
- Network protocol definitions
- Packet structures
- Common utility functions

## Communication Protocol

### UDP Packets (Control)

**Sensor Data Packet** (Robot → Server/App):
```
[packet_type][timestamp][pitch][roll][distance][battery_voltage]
```

**Control Command Packet** (Server/App → Robot):
```
[packet_type][left_motor_speed][right_motor_speed][servo_angle]
```

**Joystick Packet** (App → Robot):
```
[packet_type][x_axis][y_axis][servo_delta]
```

### TCP Stream (Vision)

**Frame Packet** (ESP32-CAM → Server/App):
```
[frame_size][frame_data][checksum]
```

## Build System

- **Framework**: ESP-IDF (CMake-based)
- **Dependency Management**: `idf_component.yml` for component dependencies
- **Configuration**: `sdkconfig` for ESP-IDF menuconfig settings
- **Environment**: `env.sh` for build environment setup

## Configuration

Key parameters configurable via `sdkconfig`:
- WiFi credentials (SSID, password)
- Network addresses (server IP, ports)
- Task priorities and stack sizes
- Buffer sizes
- Sensor calibration values
- PID tuning constants
