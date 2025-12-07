"""
Configuration file for autonomous navigation system
"""

# Network Configuration
UDP_IMU_PORT = 9001
TCP_CAMERA_PORT = 9006  # Using camera 2 since it's the working one
DISCOVERY_PORT = 9009
UDP_CONTROL_PORT = 9000

# Sensor Configuration
IMU_SAMPLE_RATE = 100  # Hz
ULTRASOUND_MAX_RANGE = 400  # cm
ULTRASOUND_MIN_RANGE = 2    # cm
ULTRASOUND_TIMEOUT = 1.0    # seconds

# Object Detection
YOLO_MODEL = "yolov8n.pt"  # Nano model for speed
DETECTION_CONFIDENCE = 0.5
DETECTION_SIZE = 640

# Navigation Parameters
SAFE_DISTANCE = 50.0        # cm - stop distance
SLOW_DISTANCE = 100.0       # cm - slow down distance
MAX_SPEED = 1.0             # magnitude (0-1)
MIN_SPEED = 0.3
TURN_RATE = 45.0            # degrees per second

# Path Planning
GRID_CELL_SIZE = 20         # cm
EXPLORATION_TIMEOUT = 300   # seconds
OBSTACLE_PADDING = 30       # cm

# Display
DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 720
FPS_UPDATE_INTERVAL = 30

# Kalman Filter Parameters
IMU_PROCESS_NOISE = 0.01
IMU_MEASUREMENT_NOISE = 0.1

# Control Output
SERVO_CENTER = 90
SERVO_RANGE = 45  # +/- degrees from center
