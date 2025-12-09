# Getting Started Guide

## Project Setup

### Prerequisites

#### Hardware
- Assembled robot with all components installed
- 3S LiPo battery (11.6V, 2200mAh) charged
- WiFi router or access point
- PC/Laptop for server mode (optional for edge mode)
- Android device for mobile app

#### Software
- Python 3.8+ (for server)
- ESP-IDF v4.4+ (for firmware)
- Android device with APK installed (for app)

### Repository Structure

```
GyroBot/
├── firmware/       # ESP32 embedded code
├── server/         # PC-based autonomy server
├── app/            # Mobile control application
├── fpga/           # FPGA edge computing code
├── hardware/       # 3D models and schematics
└── docs/           # Documentation (you are here)
```

## Firmware Setup

### 1. Install ESP-IDF

```bash
# Install ESP-IDF (follow official guide)
git clone -b v4.4 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh
```

### 2. Configure WiFi

Edit network credentials in `sdkconfig`:

```bash
cd firmware/control/
idf.py menuconfig
# Navigate to: Component config → WiFi Station
# Set SSID and password
```

### 3. Build and Flash Control Firmware

```bash
cd firmware/control/
source ../env.sh  # Load environment

# Build
idf.py build

# Flash to ESP32
idf.py -p /dev/ttyUSB0 flash

# Monitor output
idf.py -p /dev/ttyUSB0 monitor
```

### 4. Build and Flash Vision Firmware

```bash
cd firmware/vision/
idf.py build
idf.py -p /dev/ttyUSB1 flash  # Different port for ESP32-CAM
idf.py -p /dev/ttyUSB1 monitor
```

**Note**: Identify correct USB ports using `ls /dev/tty*`

## Server Setup

### 1. Install Python Dependencies

```bash
cd server/

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install requirements
pip install ultralytics opencv-python numpy torch matplotlib
```

### 2. Configure Network Settings

Edit `config.py`:

```python
UDP_CONTROL_PORT = 5000
UDP_TELEMETRY_PORT = 5001
TCP_VIDEO_PORT = 5002
```

### 3. Run Server

```bash
python main.py
```

The server will:
- Connect to robot
- Start receiving sensor data
- Display video stream
- Run object detection
- Send navigation commands (if autonomous mode)

## Mobile App Setup

### 1. Install APK

Transfer `app/bin/gyrocontroller-1.0-arm64-v8a-debug.apk` to Android device and install.

**Allow installation from unknown sources** in Android settings.

### 2. Configure App

1. Open GyroController app
2. Tap Settings (⚙️ icon)
3. Enter Robot IP address
4. Set control and telemetry ports
5. Save settings

### 3. Connect to Robot

1. Ensure device is on same WiFi network as robot
2. Tap "Connect" button
3. Wait for connection indicator (green)
4. Telemetry should start updating

### 4. Test Manual Control

1. Use joystick to move robot
2. Verify motor response
3. Test servo controls (camera tilt)
4. Check battery and sensor readings

## First Power-On

### Safety Checklist

- [ ] Battery fully charged
- [ ] All connections secure
- [ ] Robot on clear, flat surface
- [ ] Emergency stop ready (disconnect battery)
- [ ] Observe from safe distance initially

### Startup Sequence

1. **Power On**:
   - Connect battery
   - Both ESP32 LEDs should light up
   - Wait ~10 seconds for WiFi connection

2. **Verify WiFi Connection**:
   - Check serial monitor for IP address
   - Ping robot from PC: `ping <robot_ip>`

3. **Test Balance**:
   - Hold robot vertically
   - Slowly tilt forward/backward
   - Motors should respond to maintain balance
   - If oscillating, reduce Kp gain

4. **Test Communication**:
   - Start server or app
   - Verify telemetry updates
   - Send test commands (low speed)

5. **Full Operation**:
   - Place robot on floor
   - Start autonomous mode or manual control
   - Monitor behavior

## Troubleshooting

### Robot Won't Balance

**Symptoms**: Falls over immediately, oscillates wildly

**Solutions**:
- Check IMU mounting (should be level when robot is vertical)
- Calibrate IMU (run calibration routine)
- Adjust PID gains (start with lower Kp)
- Verify motor directions (both motors should push same direction for balance)
- Check battery voltage (low voltage = weak response)

### No WiFi Connection

**Symptoms**: ESP32 can't connect to network

**Solutions**:
- Verify SSID and password in `sdkconfig`
- Check router settings (2.4GHz enabled, not 5GHz only)
- Move closer to router
- Check serial monitor for error messages
- Try different channel on router

### Video Stream Not Working

**Symptoms**: No frames received on server/app

**Solutions**:
- Verify ESP32-CAM is powered (LED should blink)
- Check TCP port settings match on both sides
- Test with standalone receiver: `python server/test/tcp_image_receiver.py`
- Reduce frame rate in camera config
- Check network bandwidth (WiFi congestion)

### Motors Not Responding

**Symptoms**: No movement when commands sent

**Solutions**:
- Check DRV8833 power (10.5V rail)
- Verify motor connections (not reversed)
- Test motors with direct PWM (bypass balance loop)
- Check for motor driver thermal shutdown
- Verify ESP32 GPIO assignments

### High Latency

**Symptoms**: Slow response to commands, video lag

**Solutions**:
- Reduce distance to WiFi router
- Lower video resolution/frame rate
- Disable other devices on network
- Use 5GHz WiFi if available
- Check for packet loss: `ping -c 100 <robot_ip>`

## Calibration

### IMU Calibration

1. Place robot on perfectly flat surface
2. Run calibration script (in firmware)
3. Robot must remain stationary
4. Calibration values saved to NVS

### Motor Direction Calibration

1. Set both motors to 50% forward
2. Both wheels should rotate in same direction
3. If one is reversed, swap motor driver wires for that motor

### PID Tuning

Start with conservative values:
```
Kp = 20
Ki = 0
Kd = 0.5
```

Tuning procedure:
1. Increase Kp until oscillation starts
2. Reduce Kp by 30%
3. Add Kd to dampen oscillations
4. Add small Ki to eliminate steady-state error
5. Test with load (place object on robot)

## Next Steps

### Learn the System

1. Read documentation in `docs/` folder
2. Experiment with manual control
3. Observe autonomous mode behavior
4. Review code in each component

### Extend Functionality

Ideas for enhancement:
- Add more sensor types (IMU, encoders)
- Implement advanced SLAM algorithms
- Create custom object detection classes
- Build simulation environment
- Add telemetry logging and analysis

### Join the Community

(Add links to project forums, Discord, etc. if available)

## Support

For issues and questions:
- Check documentation in `docs/`
- Review code comments
- Search existing issues on repository
- Open new issue with detailed description

## Safety Reminders

- Always supervise robot during operation
- Test in open areas first
- Keep emergency stop accessible
- Monitor battery temperature
- Don't operate with damaged components
- Use appropriate battery charger and storage
