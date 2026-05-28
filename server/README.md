# Robot Control Server

Advanced PC server for ESP32-based robot with object detection and path mapping.

## Features

- **Real-time Object Detection**: YOLOv8-powered detection with tracking and counting
- **2D Path Mapping**: Visual path tracking with obstacle detection using IMU and ultrasound
- **Beautiful GUI**: Real-time visualization of both detection and mapping
- **Multi-device Communication**: Handles ESP32 camera, sensors, and mobile app simultaneously
- **Optimized Performance**: Efficient CPU processing for real-time operation

## Architecture

```
ESP32 Camera (Port 9006) ──TCP──> PC Server ──Process──> Mobile App (Port 9011)
ESP32 Sensors (Port 9001) ──UDP──> PC Server
Mobile App (Port 9010) ──UDP──> PC Server (Mode Control)
```

## Installation

### 1. Install Python Dependencies

```bash
pip install -r requirements.txt
```

### 2. Download YOLOv8 Model

The YOLOv8 nano model will be automatically downloaded on first run.

## Usage

### Start the Server

```bash
python main.py
```

The GUI will open showing:
- **Left Panel**: Object detection with live camera feed and detection statistics
- **Right Panel**: 2D path map showing robot movement and obstacles
- **Top Bar**: Connection status and current mode

### Network Ports

| Port | Direction | Purpose |
|------|-----------|---------|
| 9006 | ESP32 → PC | Camera images (TCP) |
| 9001 | ESP32 → PC | IMU/Ultrasound data (UDP) |
| 9000 | PC → ESP32 | Control commands (UDP) |
| 9009 | PC → ESP32 | Discovery broadcast (UDP) |
| 9010 | App → PC | Mode selection (UDP) |
| 9011 | PC → App | Processed images (TCP) |

## Modes

### Object Detection Mode
- Performs real-time object detection on camera feed
- Tracks all objects detected with counts
- Displays annotated video with bounding boxes
- Shows statistics: FPS, total detections, object counts

### 2D Path Mapping Mode
- Visualizes robot's traveled path in black
- Shows obstacles color-coded by distance (red=near, blue=far)
- Displays robot position and heading
- Tracks path length and obstacle count

## Configuration

Edit `config.py` to adjust:
- Detection confidence threshold
- Map size and scale
- JPEG quality
- Buffer sizes
- Network timeouts

## Performance

- **Object Detection**: ~10-15 FPS on CPU (i5/i7)
- **Path Mapping**: Real-time updates at sensor rate
- **Network**: Low latency UDP for sensors, reliable TCP for images

## Troubleshooting

### No Camera Connection
- Verify ESP32 is sending to correct IP and port 9006
- Check firewall settings
- Ensure ESP32 and PC are on same network

### No Sensor Data
- Check ESP32 is sending UDP to port 9001
- Verify broadcast is working (port 9009)
- Check network allows UDP broadcast

### Poor Detection Performance
- Lower confidence threshold in config.py
- Ensure adequate lighting for camera
- Consider upgrading to YOLOv8s for better accuracy (slower)

### App Not Receiving Images
- Verify mobile app connects to port 9011
- Check mode is being sent correctly on port 9010
- Monitor transmitter logs for connection status

## Advanced Features

### Custom Detection Classes
YOLOv8 detects 80 COCO classes by default. To use custom classes:
1. Train a custom YOLOv8 model
2. Replace model path in `object_detector.py`

### Path Replay
Path data is stored in memory. To save/load paths:
- Access `mapper.path_points` and `mapper.obstacle_points`
- Serialize to JSON for persistence

### Multi-Robot Support
To support multiple robots:
- Assign unique ports per robot
- Run multiple server instances
- Modify discovery to handle robot IDs

## License

MIT License - Free to use and modify

## Credits

- Object Detection: [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- Computer Vision: [OpenCV](https://opencv.org/)
