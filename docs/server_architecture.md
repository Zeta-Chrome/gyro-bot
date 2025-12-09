# Server Architecture

## Overview

The server runs on a PC/laptop and provides high-level intelligence for the robot in Server Mode. It performs computationally intensive tasks like object detection and SLAM that are too demanding for the ESP32.

## Core Components

### Main Application (`server/main.py`)

Central orchestration point that:
- Initializes all subsystems
- Coordinates data flow between components
- Manages main processing loop
- Handles user interface (if GUI enabled)

### Sensor Receiver (`sensor_receiver.py`)

**Purpose**: Receive and parse UDP packets from the robot's sensors

**Responsibilities**:
- Listen on UDP port for incoming sensor data
- Parse binary packet structures
- Validate packet integrity
- Publish sensor data to other components
- Maintain sensor data history for trend analysis

**Data Received**:
- IMU readings (pitch, roll, yaw)
- Ultrasonic distance measurements
- Battery voltage
- Robot status flags

**Threading**: Runs in separate thread to avoid blocking main loop

### TCP Image Receiver (`test/tcp_image_receiver.py`)

**Purpose**: Receive video stream from ESP32-CAM

**Responsibilities**:
- Maintain TCP connection with camera
- Receive frame packets
- Decode JPEG/raw frames
- Buffer frames for processing
- Synchronize with frame processing pipeline

**Frame Format**:
```
[4 bytes: frame_size][frame_data][2 bytes: checksum]
```

**Threading**: Separate thread for continuous reception

### Object Detector (`object_detector.py`)

**Purpose**: Real-time object detection using YOLOv8

**Key Functions**:

#### Model Loading
- Loads pre-trained YOLOv8 model (`yolov8n.pt`)
- Supports various YOLO variants (nano, small, medium, large)
- GPU acceleration if CUDA available

#### Detection Pipeline
```python
1. Receive frame from camera stream
2. Preprocess (resize, normalize)
3. Run inference
4. Post-process (NMS, confidence filtering)
5. Extract bounding boxes and class labels
6. Send to path planner and display
```

**Output Format**:
- Bounding boxes: `[x, y, width, height]`
- Class labels: Object type (person, car, obstacle, etc.)
- Confidence scores: Detection certainty
- Distance estimation: Using ultrasonic fusion

**Performance Optimizations**:
- Frame skipping during high load
- Batch processing if multiple frames queued
- Resolution scaling for faster inference
- Model quantization options

### Path Planner (`path_planner.py`)

**Purpose**: Generate navigation commands based on detection results

**Inputs**:
- Object detection bounding boxes and classes
- Current robot position and orientation
- Ultrasonic distance readings
- Goal position (if navigating to target)

**Algorithm**:

#### Obstacle Avoidance
- Convert detections to obstacle map
- Apply safety margins around obstacles
- Calculate repulsive vectors from obstacles

#### Goal Seeking
- Calculate attractive vector toward goal
- Weight by distance and priority

#### Command Generation
```python
resultant_vector = goal_vector + sum(obstacle_vectors)
target_velocity = magnitude(resultant_vector)
target_heading = angle(resultant_vector)
```

**Outputs** (sent via UDP to robot):
- Left motor speed
- Right motor speed
- Servo angle (for camera positioning)

**Safety Features**:
- Emergency stop if obstacle too close
- Speed reduction near obstacles
- Path validation before execution

### Sensor Fusion (`sensor_fusion.py`)

**Purpose**: Combine multiple sensor sources for accurate state estimation

**Fusion Algorithm**:

#### Visual Odometry
- Track feature points between frames
- Estimate camera motion (translation + rotation)
- Scale using ultrasonic distance

#### IMU Integration
- Integrate gyroscope for rotation
- Use accelerometer for tilt correction
- Complementary filter with visual data

#### State Estimation
Uses Extended Kalman Filter (EKF):
- **State Vector**: `[x, y, theta, vx, vy, omega]`
  - Position, orientation, velocities
- **Prediction**: Based on motion model
- **Update**: Correct using sensor measurements
- **Output**: Best estimate of robot pose

**Outputs**:
- Robot position (x, y)
- Heading angle (theta)
- Velocities
- Covariance (uncertainty estimate)

### Display Module (`display.py`)

**Purpose**: Visualize robot status and sensor data

**Display Elements**:
- Live video feed with detection overlays
- Bounding boxes around detected objects
- Robot telemetry (pitch, distance, battery)
- Planned path visualization
- Sensor history plots

**Technologies**:
- OpenCV for video display
- Matplotlib for plotting
- GUI framework (Tkinter/PyQt) for controls

### Configuration (`config.py`)

**Purpose**: Centralized configuration management

**Settings**:
- Network addresses (robot IP, ports)
- Model paths (YOLO weights)
- Detection thresholds
- Path planning parameters
- Display options

## Data Flow

```
ESP32 → UDP → Sensor Receiver → Sensor Fusion
                                      ↓
ESP32-CAM → TCP → Image Receiver → Object Detector
                                      ↓
                            Path Planner → UDP → ESP32
                                      ↓
                                  Display
```

## Server Operation Modes

### Monitoring Mode
- Receives and displays sensor data
- Shows video stream
- No control commands sent
- Used for debugging and observation

### Autonomous Mode
- Full processing pipeline active
- Object detection running
- Path planning enabled
- Control commands sent to robot
- Used for autonomous navigation

### Manual Override
- Joystick input from app takes precedence
- Server continues monitoring
- Can provide navigation suggestions
- Safety features remain active

## Performance Considerations

### Latency Reduction
- Direct UDP communication (no TCP overhead for control)
- Frame skipping when processing falls behind
- Priority queuing for critical commands

### CPU/GPU Utilization
- YOLOv8 inference on GPU if available
- Parallel processing of video and sensors
- Async I/O for network operations

### Reliability
- Automatic reconnection on network failure
- Packet loss handling
- Timeout detection
- Graceful degradation when sensors fail

## Testing

### Test Scripts (`server/test/`)

#### Packet Testing (`packets.py`)
- Generate synthetic sensor packets
- Test packet parsing
- Validate data integrity

#### TCP Image Testing (`tcp_image_receiver.py`)
- Standalone video stream receiver
- Test frame decoding
- Measure frame rate and latency

## Dependencies

Key Python libraries:
- `ultralytics`: YOLOv8 implementation
- `opencv-python`: Image processing and display
- `numpy`: Numerical operations
- `torch`: PyTorch for model inference
- `matplotlib`: Data visualization
- Standard library: `socket`, `threading`, `struct`
