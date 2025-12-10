# Hardware Architecture

## System Components

### Processing Units

#### ESP32 (Main MCU)
- **Role**: Primary controller for balance, motor control, and sensor coordination
- **Responsibilities**:
  - PID-based self-balancing algorithm
  - Motor PWM control via DRV8833
  - Sensor data acquisition (IMU, ultrasonic)
  - Network communication (UDP/TCP)
  - Real-time task scheduling
- **Power**: 5V from buck converter

#### ESP32-CAM
- **Sensor**: OV7670 camera module
- **Role**: Visual data acquisition and streaming
- **Responsibilities**:
  - Video frame capture
  - TCP-based frame streaming
  - Visual odometry data processing
- **Power**: 5V from buck converter
- **Mount**: Servo-controlled gimbal for pitch adjustment

#### FPGA (Co-processor)
- **Role**: Edge computing for vision algorithms
- **Responsibilities**:
  - Real-time object detection inference
  - SLAM processing
  - Autonomous decision making (Edge Mode)
- **Location**: Third stack (top layer)
- **Interface**: High-speed communication with ESP32

### Sensors

#### MPU6050 (IMU)
- **Type**: 6-axis gyroscope and accelerometer
- **Sampling Rate**: High-frequency for real-time balance
- **Primary Function**: Pitch angle measurement for balance control
- **Secondary Function**: Sensor fusion for autonomous navigation
- **Power**: 3.3V from ESP32 regulator
- **Interface**: I2C

#### HC-SR04 (Ultrasonic Distance Sensor)
- **Range**: 2cm - 400cm
- **Purpose**: Accurate distance measurement to compensate for monocular camera depth limitations
- **Use Cases**:
  - Obstacle detection
  - Distance verification for visual odometry
  - Collision avoidance
- **Power**: 5V from buck converter
- **Interface**: GPIO trigger/echo pins

### Actuators

#### Motors (2x)
- **Specifications**:
  - Voltage: 12V nominal (10.5V actual operating voltage)
  - Speed: 150 RPM
  - Torque: 2 kg·cm per motor
- **Configuration**: Differential drive for forward/backward and turning
- **Purpose**: Propulsion and balance correction

#### MG90 Servo
- **Type**: Micro servo motor
- **Range**: ~180° rotation
- **Purpose**: Camera pitch control
- **Function**: Adjusts vertical field of view by tilting camera up/down
- **Power**: 5V from buck converter
- **Control**: PWM from ESP32

### Power System

#### Main Battery
- **Type**: LiPo 3S
- **Capacity**: 2200mAh
- **Voltage**: 11.6V nominal
- **Discharge Rate**: 80C (high current capability)
- **Location**: Base stack

#### Buck Converter #1 (5V Rail)
- **Input**: 11.6V from battery
- **Output**: 5V @ 6A max
- **Powers**:
  - ESP32 main controller
  - ESP32-CAM
  - HC-SR04 ultrasonic sensor
  - MG90 servo
  - MPU6050 (via ESP32 3.3V regulator)

#### Buck Converter #2 (Motor Rail)
- **Input**: 11.6V from battery
- **Output**: 10.5V (regulated for motor driver)
- **Location**: Second stack
- **Powers**: DRV8833 motor driver only

### Motor Driver

#### DRV8833
- **Configuration**: Dual H-bridge
- **Voltage**: 10.5V input (max 10.8V rated)
- **Current**: 1.5A per channel
- **Channels**: 2 (one per motor)
- **Control**: PWM from ESP32 for speed and direction
- **Protection**: Integrated current limiting and thermal shutdown

## Physical Structure

### Three-Stack Design

#### Base Stack
- Motor mounts (3D printed - `motor_mount.stl`)
- Motors with integrated gearboxes
- Main battery compartment
- Wheel assemblies

#### Second Stack
- Main circuit board with ESP32
- DRV8833 motor driver
- Buck converters (both 5V and 10.5V)
- Power distribution
- Wiring management

#### Third Stack
- ESP32-CAM module
- MG90 servo with camera gimbal
- FPGA module
- Antenna/communication components

### 3D Printed Components

All models available in `hardware/model_design/`:
- `GyroBot-Base.stl`: Main chassis and mounting platform
- `GyroBot-MotorMount.stl`: Motor mounting brackets
- `GyroBot-Parts.stl`: Additional structural components
- `GyroBot.FCStd`: FreeCAD source file for modifications

## Power Distribution

```
Battery (11.6V 2200mAh)
├── Buck #1 (5V 6A)
│   ├── ESP32 → 3.3V Regulator → MPU6050
│   ├── ESP32-CAM
│   ├── HC-SR04
│   └── MG90 Servo
└── Buck #2 (10.5V)
    └── DRV8833 Motor Driver
        ├── Motor Left
        └── Motor Right
```

## Wiring Considerations

- Motor driver isolated on separate power rail to reduce noise
- IMU powered from ESP32 3.3V regulator for stable reference
- Camera and servo on clean 5V rail for consistent operation
- High-current paths (motor driver) kept short and direct
- Signal lines separated from power lines where possible
