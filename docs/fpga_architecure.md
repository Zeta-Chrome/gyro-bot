# FPGA Edge Computing

## Overview

The FPGA serves as a co-processor for edge computing, enabling real-time autonomous operation without dependency on a PC or network connection. This is the key differentiator for deploying the robot in environments where low latency and reliability are critical.

**Location**: Third stack (top layer) of robot chassis

## Purpose

### Why FPGA?

**Advantages over ESP32**:
- Parallel processing (multiple operations simultaneously)
- Deterministic timing (no OS overhead)
- Hardware acceleration for vision algorithms
- Higher throughput for neural network inference

**Advantages over Cloud/Server**:
- No network latency (typically 10-100ms saved)
- Operation in areas without WiFi
- No bandwidth limitations
- Privacy (data stays on-device)
- Lower power than GPU

## Responsibilities in Edge Mode

### Primary Functions

#### 1. JPEG Compression
- Receive raw frames from ESP32-CAM (RGB/YUV format)
- Hardware-accelerated JPEG encoding
- Much faster than software compression on ESP32-CAM
- Reduces bandwidth requirements for transmission
- Typical compression: 640x480 raw (~900KB) → JPEG (~30-60KB)

#### 2. Object Detection
- Decode JPEG frames (if needed for processing)
- Run YOLOv8 inference (hardware accelerated)
- Extract bounding boxes and classifications
- Real-time detection at 15-30 FPS

#### 3. Path Planning & Obstacle Avoidance
- Analyze object detection results
- Generate obstacle map from detected objects
- Calculate safe navigation paths
- Apply potential field or A* algorithm
- Compute velocity and steering commands

#### 4. Autonomous Navigation
- Integrate all sensor inputs (vision, IMU, ultrasonic)
- Make real-time navigation decisions
- Execute complete autonomy stack on-device
- Issue motor commands directly to ESP32

### Secondary Functions
- Sensor data preprocessing
- Data logging (if storage available)
- Communication with ESP32 via high-speed interface

## Hardware Architecture

### FPGA Configuration

**Expected Specifications** (typical for robotics FPGA):
- Logic Elements: 50K-100K LUTs
- Memory: 5-10 Mb block RAM
- DSP Blocks: 100-200 for arithmetic operations
- I/O: High-speed serial (SPI/I2C/UART) to ESP32

### Interface with ESP32

**Communication Protocol**:
- **High-speed SPI**: 10-40 Mbps for frame transfer
- **UART**: Control commands and status
- **Shared Interrupts**: Signal processing completion

**Data Flow**:
```
ESP32-CAM → Raw Frames → FPGA
FPGA → Detection Results → ESP32
ESP32 → Sensor Data → FPGA
FPGA → Motor Commands → ESP32
```

## Software Architecture

### HDL Modules

#### JPEG Encoder
- **Input**: Raw image data from ESP32-CAM (RGB565 or YUV422)
- **Processing**:
  - Color space conversion (RGB → YCbCr)
  - 8x8 block DCT (Discrete Cosine Transform)
  - Quantization
  - Huffman encoding
- **Output**: Compressed JPEG bitstream
- **Performance**: 10-30x faster than ESP32 software encoding
- **Benefit**: Reduces camera processing load and transmission bandwidth

#### Frame Buffer Controller
- Receives JPEG frames via SPI
- Manages frame storage in FPGA RAM
- Provides data to inference pipeline
- Handles frame rate synchronization

#### Neural Network Accelerator
- Hardware implementation of YOLOv8 layers
- Convolution engines
- Activation functions
- Pooling operations
- Quantized weights (INT8 or lower)

**Optimization Strategies**:
- Layer fusion (combine operations)
- Weight pruning (remove unnecessary connections)
- Quantization (reduce precision for speed)
- Pipelining (parallel layer processing)

#### Feature Extractor (for SLAM)
- Detect visual keypoints (corners, edges)
- Extract descriptors
- Match features between frames
- Estimate camera motion

#### Obstacle Avoidance Engine
- Convert detection bounding boxes to spatial coordinates
- Build local occupancy grid
- Apply safety margins around obstacles
- Generate repulsive force vectors
- Real-time collision prediction

#### Path Planning Engine
- Maintain goal waypoint queue
- A* or Dijkstra's algorithm in hardware
- Combine with obstacle avoidance
- Dynamic replanning on obstacle detection
- Smooth trajectory generation

#### Autonomous Navigation Controller
- Convert path plan to motor commands
- Apply constraints (max speed, acceleration)
- Interface with ESP32 control loop

## Edge Mode Operation

### Startup Sequence

1. **FPGA Configuration**:
   - Load bitstream from flash
   - Initialize hardware modules
   - Self-test

2. **ESP32 Handshake**:
   - Establish SPI connection
   - Synchronize clocks
   - Exchange configuration

3. **Model Loading**:
   - Load neural network weights
   - Verify checksum
   - Ready signal to ESP32

### Processing Pipeline

```
Raw Frame (ESP32-CAM)
    ↓
JPEG Encoding (FPGA - Hardware Accelerated)
    ↓
JPEG Frame Transfer (SPI)
    ↓
JPEG Decode (FPGA - if needed for processing)
    ↓
Object Detection (FPGA - YOLOv8)
    ↓
Obstacle Mapping (FPGA)
    ↓
Path Planning (FPGA)
    ↓
Autonomous Navigation (FPGA)
    ↓
Motor Commands (FPGA → ESP32)
    ↓
Balance + Execute (ESP32)
```

**Latency Budget**:
- JPEG encoding: 10-20ms
- Frame transfer: 5-10ms
- Object detection: 20-40ms
- Path planning: 5-10ms
- Navigation: 2-5ms
- Total: 42-85ms (vs 150-250ms for server mode with software JPEG)

### Autonomous Decision Making

The FPGA runs the full autonomy stack:

1. **Image Processing**: Hardware JPEG encoding for efficiency
2. **Perception**: What objects are nearby? (Object detection)
3. **Mapping**: Where are obstacles? (Spatial representation)
4. **Planning**: What path should I take? (Path planning with obstacle avoidance)
5. **Control**: What motor speeds achieve that path? (Navigation controller)

All decisions made locally in real-time with minimal latency.

## Performance Optimization

### Neural Network Optimization

**Model Compression**:
- Use YOLOv8n (nano) or custom lightweight model
- Quantization: FP32 → INT8 (4x speedup)
- Pruning: Remove 30-50% of weights
- Knowledge distillation: Smaller model trained from larger

**Hardware Acceleration**:
- Dedicated convolution engines
- Parallel MAC (multiply-accumulate) units
- On-chip memory for weights (avoid external memory latency)

### JPEG Encoding Optimization

**Hardware Advantages**:
- Parallel 8x8 DCT blocks processing
- Pipelined quantization and encoding
- Dedicated Huffman encoder
- Sustained throughput: 30-60 FPS at VGA resolution

### Obstacle Avoidance Optimization

### Obstacle Avoidance Optimization

**Spatial Processing**:
- Fast occupancy grid updates
- Distance transform in hardware
- Efficient nearest-obstacle queries
- Vector field generation

### Path Planning Optimization

**Algorithm Selection**:
- Lightweight A* with limited search depth
- Dynamic window approach for reactive planning
- Potential field methods (low computational cost)
- Hybrid approach: global plan + local reactive

### Resource Utilization

Target allocation:
- 30% LUTs: JPEG encoding/decoding
- 40% LUTs: Neural network inference
- 15% LUTs: Obstacle avoidance & path planning
- 10% LUTs: Navigation control logic
- 5% LUTs: I/O and communication

## Development Tools

### HDL Development
- **Language**: Verilog or VHDL
- **Synthesis**: Vendor tools (Xilinx Vivado, Intel Quartus)
- **Simulation**: ModelSim, Icarus Verilog
- **Verification**: Cocotb for Python-based testbenches

### High-Level Synthesis (Optional)
- **Tools**: Vitis HLS, LegUp
- **Language**: C/C++ → HDL
- **Use Case**: Rapid prototyping of algorithms

### Neural Network Deployment
- **Frameworks**: 
  - Xilinx Vitis AI
  - Intel OpenVINO
  - Custom toolchain
- **Workflow**: PyTorch/ONNX → Quantized → FPGA bitstream

## Comparison: Server Mode vs Edge Mode

| Aspect | Server Mode | Edge Mode (FPGA) |
|--------|-------------|------------------|
| Latency | 150-250ms | 42-85ms |
| JPEG Encoding | Software (ESP32) | Hardware (FPGA) |
| Model Complexity | Full YOLOv8 | Lightweight/Quantized |
| Obstacle Avoidance | Server-side | FPGA real-time |
| Navigation | Server-side | FPGA autonomous |
| Network Dependency | Required | None |
| Power | Low (robot) + High (PC) | Medium (robot only) |
| Deployment | WiFi range | Anywhere |
| Accuracy | High | Medium-High |
| Development Complexity | Low (Python) | High (HDL) |

## Current Status

The FPGA integration is part of the project roadmap. The hardware slot is prepared on the third stack, and the ESP32 firmware includes interfaces for FPGA communication.

**Next Steps**:
1. FPGA board selection
2. JPEG encoder implementation and optimization
3. Communication protocol implementation
4. Neural network porting and quantization
5. Obstacle avoidance algorithm implementation
6. Path planning integration
7. Full autonomous navigation stack
8. Integration testing

## Benefits for Real-World Deployment

### Use Cases Enabled by Edge Computing

1. **Search and Rescue**:
   - Operate in buildings without WiFi
   - Low-latency obstacle avoidance
   - Navigate debris fields autonomously

2. **Warehouse Automation**:
   - Reliable operation without cloud dependency
   - Predictable latency for safety
   - Multiple robots without network congestion

3. **Exploration**:
   - Remote areas (caves, wilderness)
   - No infrastructure required
   - Extended battery life (no WiFi)

4. **Education/Research**:
   - Demonstrate edge AI concepts
   - Benchmark FPGA vs CPU/GPU performance
   - Hardware acceleration learning platform
