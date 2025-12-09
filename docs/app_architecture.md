# Mobile Application

## Overview

The mobile app provides manual control and monitoring capabilities for the robot. Built with Kivy/KivyMD framework, it runs on Android devices and communicates with the robot over WiFi.

**Built APK**: `app/bin/gyrocontroller-1.0-arm64-v8a-debug.apk`

## Architecture

### Main Application (`app/main.py`)

**Purpose**: Application entry point and UI coordination

**Responsibilities**:
- Initialize Kivy app
- Load UI layouts
- Create widget instances
- Manage app lifecycle
- Handle screen navigation

**Key Features**:
- Custom app icon (`images/app_icon.png`)
- Material Design theme (KivyMD)
- Responsive layout for different screen sizes

### Network Module (`app/network/`)

#### Server Communication (`server.py`)

**Purpose**: Handle all network communication with robot

**UDP Client**:
- Send control commands to robot
- Receive sensor telemetry
- Non-blocking socket operations
- Automatic reconnection

**TCP Client**:
- Receive video stream (optional feature)
- Display live camera feed in app

**Packet Formats**:

Control Command Packet:
```python
struct.pack('Bhhh', 
    packet_type,      # 1 byte
    left_motor,       # 2 bytes (signed)
    right_motor,      # 2 bytes (signed)
    servo_angle       # 2 bytes (signed)
)
```

Sensor Data Packet (received):
```python
struct.unpack('Bffffhh',
    packet_type,      # Command type
    pitch,            # IMU pitch angle
    roll,             # IMU roll angle  
    yaw,              # IMU yaw angle
    distance,         # Ultrasonic reading
    battery,          # Battery voltage
    status            # Status flags
)
```

### Widgets (`app/widgets/`)

Custom UI components for robot control:

#### Joystick (`joystick.py`)

**Purpose**: Virtual joystick for robot movement

**Visual Design**:
- Circular boundary
- Draggable knob (`images/slider_knob.png`)
- Returns to center when released
- Visual feedback during drag

**Output**:
- **X-axis**: -1.0 (full left) to +1.0 (full right)
- **Y-axis**: -1.0 (full back) to +1.0 (full forward)

**Mapping to Motor Commands**:
```python
forward_speed = y_axis * MAX_SPEED
turn_rate = x_axis * MAX_TURN_RATE

left_motor = forward_speed - turn_rate
right_motor = forward_speed + turn_rate
```

**Update Rate**: 30-60 Hz for smooth control

#### Servo Control (`servo_control.py`)

**Purpose**: Adjust camera angle

**UI Elements**:
- Slider for precise angle selection (0-180°)
- Up/Down buttons for incremental adjustment
- Current angle display

**Features**:
- Smooth angle transitions
- Safety limits (prevent mechanical damage)
- Visual indicator of current position

#### Settings Panel (`settings.py`)

**Purpose**: Configure app and robot parameters

**Settings**:
- **Network Configuration**:
  - Robot IP address
  - UDP control port
  - UDP telemetry port
  - TCP video port
- **Control Settings**:
  - Joystick sensitivity
  - Maximum speed limits
  - Turn rate multiplier
  - Servo step size
- **Display Settings**:
  - Show video stream
  - Telemetry refresh rate
  - Theme selection

**Persistence**: Settings saved to local storage

#### Dropdown Menu (`dropdown.py`)

**Purpose**: Reusable dropdown selection widget

**Usage**:
- Mode selection (Manual/Autonomous/Edge)
- Network interface selection
- Theme selection

#### Triangle Button (`triangle_button.py`)

**Purpose**: Directional buttons (servo up/down)

**Visual**: Triangle-shaped button pointing direction
**Interaction**: Touch and hold for continuous movement

## User Interface

### Main Screen Layout

```
┌─────────────────────────────┐
│        App Title            │
│      [Settings] [Mode]      │
├─────────────────────────────┤
│                             │
│   [Video Feed Area]         │
│   (if enabled)              │
│                             │
├─────────────────────────────┤
│  Pitch: XX°  Distance: XXcm │
│  Battery: X.XV              │
├─────────────────────────────┤
│                             │
│         [Joystick]          │
│            (○)              │
│                             │
├─────────────────────────────┤
│  Camera Control             │
│    ▲  [Slider]  ▼          │
└─────────────────────────────┘
```

### Control Flow

1. **App Launch**:
   - Load settings from storage
   - Initialize network module
   - Connect to robot
   - Start telemetry receiver

2. **Manual Control**:
   - User touches joystick
   - Calculate motor speeds
   - Pack into UDP packet
   - Send to robot
   - Repeat at 30+ Hz

3. **Telemetry Display**:
   - Receive UDP packets from robot
   - Update UI labels
   - Show warnings (low battery, high tilt)
   - Log data if enabled

4. **Mode Switching**:
   - User selects mode from dropdown
   - Send mode change command
   - Update UI for selected mode
   - Disable/enable appropriate controls

## Operation Modes

### Manual Mode
- Joystick fully active
- Direct control over motors and servo
- Real-time telemetry display
- Video feed optional

### Autonomous Mode (Monitor)
- Joystick disabled
- Display autonomous navigation status
- Show detected objects overlay
- Emergency stop button available

### Edge Mode (Monitor)
- Similar to autonomous mode
- Indicates processing on FPGA
- Lower latency metrics displayed

## Build System

### Buildozer Configuration (`buildozer.spec`)

**Key Settings**:
- **Package**: com.gyrobot.controller
- **Version**: 1.0
- **Requirements**: 
  - kivy
  - kivymd
  - python3
  - socket (built-in)
- **Permissions**:
  - INTERNET
  - ACCESS_NETWORK_STATE
  - ACCESS_WIFI_STATE
- **Orientation**: Portrait
- **Android API**: 31 (Android 12)
- **Architecture**: arm64-v8a

### Building

```bash
# Install buildozer
pip install buildozer

# Build APK
cd app/
buildozer android debug

# Output: bin/gyrocontroller-1.0-arm64-v8a-debug.apk
```

## Features

### Real-time Control
- Low-latency UDP communication (~10-30ms)
- Smooth joystick response
- Immediate feedback

### Safety Features
- Connection status indicator
- Timeout detection (stop robot if no response)
- Battery level warnings
- Emergency stop button
- Tilt angle warnings

### User Experience
- Intuitive joystick controls
- Visual feedback for all actions
- Persistent settings
- Material Design aesthetics
- Haptic feedback on button press (optional)

## Testing

### Development Testing
```bash
# Run on desktop (for development)
python main.py
```

### Device Testing
1. Install APK on Android device
2. Connect device to same WiFi as robot
3. Configure robot IP in settings
4. Test connectivity
5. Test joystick control
6. Verify telemetry reception

## Future Enhancements

Potential additions:
- Video recording capability
- Waypoint navigation interface
- Sensor data graphing
- Multiple robot support
- Bluetooth connectivity option
- Voice commands
