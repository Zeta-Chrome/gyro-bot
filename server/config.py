"""
Configuration settings for robot server
"""

# Network ports
UDP_IMU_PORT = 9001          # Receive IMU/ultrasound from ESP32
UDP_CONTROL_PORT = 9000      # Send control to ESP32
UDP_MODE_PORT = 9010         # Receive mode from mobile app
TCP_CAMERA_PORT = 9006       # Receive camera from ESP32
TCP_OUTPUT_PORT = 9011       # Send processed images to mobile app
DISCOVERY_PORT = 9009        # Discovery broadcast

# Camera settings
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480

# Object detection settings
DETECTION_CONFIDENCE = 0.45
DETECTION_NMS_THRESHOLD = 0.4
DETECTION_MODEL = 'yolov8n'  # nano model for speed

# Path mapping settings
MAP_SIZE = 800  # pixels
MAP_SCALE = 50  # pixels per meter
MAX_DISTANCE = 5.0  # meters
PATH_COLOR = (0, 0, 0)  # black
ROBOT_COLOR = (0, 255, 0)  # green
OBSTACLE_NEAR_COLOR = (0, 0, 255)  # red
OBSTACLE_FAR_COLOR = (255, 0, 0)  # blue

# Buffer settings
IMU_BUFFER_SIZE = 100
ULTRASOUND_BUFFER_SIZE = 50
POSITION_BUFFER_SIZE = 1000

# JPEG quality
JPEG_QUALITY = 85
