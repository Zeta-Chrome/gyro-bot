# Build Instructions

Complete guide to building and assembling the autonomous self-balancing robot from scratch.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Hardware Assembly](#hardware-assembly)
3. [3D Printing](#3d-printing)
4. [Electronics Assembly](#electronics-assembly)
5. [Firmware Setup](#firmware-setup)
6. [Server Setup](#server-setup)# Build Instructions

Complete guide to building and assembling the autonomous self-balancing robot from scratch.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Hardware Assembly](#hardware-assembly)
3. [3D Printing](#3d-printing)
4. [Electronics Assembly](#electronics-assembly)
5. [Firmware Setup](#firmware-setup)
6. [Server Setup](#server-setup)
7. [Mobile App Setup](#mobile-app-setup)
8. [Initial Testing](#initial-testing)
9. [Calibration](#calibration)

---

## Prerequisites

### Tools Required

#### Mechanical Tools
- Soldering iron and solder
- Wire strippers
- Small screwdrivers (Phillips and flat head)
- Hex key set
- Multimeter
- Heat shrink tubing and heat gun
- Helping hands/PCB holder
- Diagonal cutters

#### Software Tools
- Computer with USB ports
- USB to TTL serial adapter (or USB cables for ESP32)
- Text editor or IDE (VS Code recommended)
- Git for version control

### Parts List

#### Microcontrollers
- [ ] 1x ESP32 DevKit (main controller)
- [ ] 1x ESP32-CAM module with OV7670 camera
- [ ] 1x FPGA board (TBD based on requirements)

#### Sensors
- [ ] 1x MPU6050 6-axis IMU module
- [ ] 1x HC-SR04 ultrasonic distance sensor
- [ ] Connecting wires (Dupont cables)

#### Motors & Drivers
- [ ] 2x DC geared motors (12V, 150 RPM, 2kg·cm torque)
- [ ] 1x DRV8833 dual motor driver module
- [ ] 2x Wheels (compatible with motor shaft)

#### Servo
- [ ] 1x MG90S micro servo motor

#### Power System
- [ ] 1x LiPo battery 3S (11.1V, 2200mAh, 80C)
- [ ] 1x LiPo battery charger (with balance charging)
- [ ] 1x Buck converter 5V 6A
- [ ] 1x Buck converter adjustable (for 10.5V)
- [ ] 1x XT60 connector (battery connector)
- [ ] 1x Power switch
- [ ] Battery voltage alarm (optional but recommended)

#### Electronics
- [ ] Perfboard or custom PCB
- [ ] 22-24 AWG wire (red and black for power)
- [ ] 26-28 AWG wire (for signals)
- [ ] Pin headers (male and female)
- [ ] JST connectors (optional, for modular connections)
- [ ] Heat shrink tubing (various sizes)
- [ ] Electrical tape

#### Mechanical Parts
- [ ] 3D printed chassis (files in `hardware/model_design/`)
- [ ] M3 screws and nuts (various lengths: 8mm, 12mm, 16mm)
- [ ] M2.5 screws (for motor mounts)
- [ ] Standoffs (M3, 20mm and 40mm)
- [ ] Velcro straps or zip ties

---

## Hardware Assembly

### Step 1: 3D Printing

#### Print the Chassis Components

Navigate to `hardware/model_design/` and print the following STL files:

1. **GyroBot-Base.stl**
   - Material: PLA or PETG
   - Layer height: 0.2mm
   - Infill: 20-30%
   - Supports: Yes
   - Print time: ~4-6 hours

2. **GyroBot-MotorMount.stl** (print 2x)
   - Material: PLA or PETG
   - Layer height: 0.2mm
   - Infill: 30-40% (needs strength)
   - Supports: Minimal
   - Print time: ~1-2 hours each

3. **GyroBot-Parts.stl**
   - Contains: Servo mount, camera bracket, standoffs
   - Material: PLA
   - Layer height: 0.2mm
   - Infill: 20%
   - Print time: ~2-3 hours

#### Post-Processing
- Remove supports carefully
- Clean up rough edges with sandpaper
- Test fit all screw holes (drill out if tight)
- Dry fit motors in motor mounts

### Step 2: Motor Assembly

#### Mount Motors to Motor Mounts

1. Insert motor into motor mount bracket
2. Align screw holes
3. Secure with M2.5 screws (4 per motor)
4. Ensure motor shaft rotates freely
5. Attach wheels to motor shafts

#### Attach Motor Mounts to Base

1. Position motor mounts on base chassis
2. Align mounting holes
3. Use M3 x 16mm screws and nuts
4. Tighten firmly but don't strip plastic
5. Verify both wheels are level and aligned

### Step 3: Create the Three-Stack Structure

#### Base Stack (Layer 1)
- Chassis with motors attached
- Battery mounting area (secure with velcro)

#### Second Stack (Layer 2)
- Mount on standoffs (40mm M3 standoffs from base)
- Will hold main circuit board
- Buck converters mounted here

#### Third Stack (Layer 3)
- Mount on standoffs (20mm M3 standoffs from layer 2)
- Camera mount with servo
- FPGA mounting area

---

## Electronics Assembly

### Step 4: Power Distribution Board

Create the power distribution circuit on perfboard:

#### Power Distribution Schematic

```
Battery (11.1V) → Power Switch → Split
                                  ├── Buck #1 (5V 6A)
                                  │   ├→ ESP32
                                  │   ├→ ESP32-CAM
                                  │   ├→ HC-SR04
                                  │   └→ Servo
                                  │
                                  └── Buck #2 (10.5V)
                                      └→ DRV8833 Motor Driver
```

#### Assembly Steps

1. **Solder Buck Converters**
   - Buck #1: Set output to exactly 5.0V (measure with multimeter)
   - Buck #2: Set output to 10.5V
   - Solder input wires (thick, 22 AWG)
   - Solder output wires with polarity markings

2. **Create Power Rails**
   - Use perfboard to create 5V and GND rails
   - Solder thick wire for current capacity
   - Add capacitors (100µF) for noise filtering

3. **Wire Power Switch**
   - Connect battery positive to switch
   - Connect switch output to distribution point
   - Test continuity with multimeter

4. **Wire XT60 Battery Connector**
   - Solder to power input (before switch)
   - Verify polarity (red = positive, black = negative)
   - Use heat shrink on connections

### Step 5: Main Controller Wiring

#### ESP32 Connections

Create a wiring harness for the ESP32:

**Motor Driver (DRV8833)**
```
ESP32 GPIO → DRV8833
GPIO 12    → AIN1 (Motor A forward)
GPIO 13    → AIN2 (Motor A backward)
GPIO 14    → BIN1 (Motor B forward)
GPIO 27    → BIN2 (Motor B backward)
```

**IMU (MPU6050)**
```
ESP32 → MPU6050
3.3V  → VCC
GND   → GND
GPIO 21 → SDA
GPIO 22 → SCL
```

**Ultrasonic Sensor (HC-SR04)**
```
ESP32 → HC-SR04
5V    → VCC
GND   → GND
GPIO 18 → TRIG
GPIO 19 → ECHO
```

**Servo (MG90S)**
```
ESP32 → Servo
5V    → VCC (red wire)
GND   → GND (brown/black wire)
GPIO 25 → Signal (orange/yellow wire)
```

**ESP32-CAM Communication** (optional, if not standalone)
```
ESP32 → ESP32-CAM
GPIO 16 (RX2) → TX
GPIO 17 (TX2) → RX
GND → GND
```

#### Soldering Tips
- Tin all wires before connecting
- Use heat shrink on all connections
- Keep wire runs short and neat
- Label all wires with tape/markers
- Test continuity after each connection

### Step 6: Motor Driver Setup

#### DRV8833 Connections

1. **Power Input**
   - Connect VM to 10.5V from Buck #2
   - Connect GND to common ground
   - Add 100µF capacitor across VM and GND (close to driver)

2. **Motor Outputs**
   - Connect Motor A to AOUT1 and AOUT2
   - Connect Motor B to BOUT1 and BOUT2
   - Polarity doesn't matter (can reverse in code)

3. **Logic Power**
   - Connect VCC to 3.3V from ESP32
   - Connect GND to common ground

### Step 7: Camera Module Assembly

#### ESP32-CAM Setup

1. **Program Before Installation**
   - ESP32-CAM needs USB-TTL adapter for programming
   - Connect: GND-GND, 5V-5V, U0R-TX, U0T-RX
   - Connect GPIO 0 to GND for programming mode
   - Upload firmware
   - Remove GPIO 0 connection for normal operation

2. **Mount Camera and Servo**
   - Attach servo to camera mount (printed part)
   - Attach ESP32-CAM to servo arm
   - Connect servo signal wire
   - Secure with small screws

3. **Power Connection**
   - 5V from Buck #1 to ESP32-CAM 5V pin
   - Common ground

---

## Firmware Setup

### Step 8: Install Development Environment

#### Install ESP-IDF

```bash
# Clone ESP-IDF repository
cd ~
git clone -b v4.4.6 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32

# Add to shell profile (bash/zsh)
echo ". ~/esp-idf/export.sh" >> ~/.bashrc
source ~/.bashrc
```

#### Clone Project Repository

```bash
git clone https://github.com/Zeta-Chrome/gyro-bot.git
cd gyro-bot
```

### Step 9: Configure and Build Control Firmware

```bash
cd firmware/control

# Load ESP-IDF environment
source ../env.sh
# Or if not using env.sh:
# . ~/esp-idf/export.sh

# Configure project
idf.py menuconfig
```

#### Configuration Steps in menuconfig

1. **WiFi Configuration**
   - Navigate: `Component config` → `WiFi Station Configuration`
   - Set `WiFi SSID`: Your network name
   - Set `WiFi Password`: Your network password

2. **Network Configuration**
   - Set server IP address (your PC's IP)
   - Set UDP control port (default: 5000)
   - Set UDP telemetry port (default: 5001)

3. **IMU Configuration**
   - Set I2C pins (default: SDA=21, SCL=22)
   - Set sample rate (default: 100Hz)
   - Set Mahony filter gains (Kp=2.0, Ki=0.01)

4. **Motor Configuration**
   - Set PWM frequency (default: 20kHz)
   - Set motor GPIO pins
   - Set maximum PWM duty cycle

5. **PID Tuning**
   - Set initial PID values (Kp=20, Ki=0, Kd=0.5)
   - These will need tuning after assembly

6. Save and exit

#### Build and Flash

```bash
# Build firmware
idf.py build

# Identify ESP32 port
ls /dev/tty*  # Look for /dev/ttyUSB0 or similar

# Flash to ESP32
idf.py -p /dev/ttyUSB0 flash

# Monitor output (Ctrl+] to exit)
idf.py -p /dev/ttyUSB0 monitor
```

Verify in monitor output:
- WiFi connection successful
- IP address displayed
- IMU initialized
- Motor driver ready

### Step 10: Configure and Build Vision Firmware

```bash
cd ../vision

# Configure (similar WiFi setup)
idf.py menuconfig

# Set camera configuration
# - Frame size: QVGA (320x240) or VGA (640x480)
# - JPEG quality: 12 (lower = better quality, larger size)
# - Frame rate: 15-30 FPS

# Build and flash
idf.py build
idf.py -p /dev/ttyUSB1 flash  # Note: different port
idf.py -p /dev/ttyUSB1 monitor
```

Verify:
- Camera initialization successful
- TCP server started
- Streaming ready

---

## Server Setup

### Step 11: Python Environment

```bash
cd ../../server

# Create virtual environment
python3 -m venv venv

# Activate (Linux/Mac)
source venv/bin/activate
# Windows:
# venv\Scripts\activate

# Install dependencies
pip install --upgrade pip
pip install ultralytics opencv-python numpy torch matplotlib
```

### Step 12: Configure Server

Edit `config.py`:

```python
# Network Configuration
ROBOT_IP = "192.168.1.100"  # ESP32 IP from serial monitor
UDP_CONTROL_PORT = 5000
UDP_TELEMETRY_PORT = 5001
TCP_VIDEO_PORT = 5002

# Detection Configuration
YOLO_MODEL = "yolov8n.pt"  # Nano model (fastest)
CONFIDENCE_THRESHOLD = 0.5
IOU_THRESHOLD = 0.4

# Display Configuration
SHOW_VIDEO = True
SHOW_DETECTIONS = True
```

### Step 13: Download YOLO Model

```bash
# Model will auto-download on first run, or manual:
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

### Step 14: Test Server

```bash
python main.py
```

Expected output:
- Connecting to robot...
- Receiving telemetry
- Video stream active
- Press 'q' to quit

---

## Mobile App Setup

### Step 15: Install App on Android

#### Option A: Pre-built APK (Quick)

1. Transfer `app/bin/gyrocontroller-1.0-arm64-v8a-debug.apk` to Android device
2. Enable "Install from Unknown Sources" in Android settings
3. Open file manager and install APK
4. Open GyroController app

#### Option B: Build from Source

```bash
cd app/

# Install Buildozer
pip install buildozer

# Install dependencies (Ubuntu/Debian)
sudo apt update
sudo apt install -y git zip unzip openjdk-11-jdk python3-pip autoconf libtool pkg-config zlib1g-dev libncurses5-dev libncursesw5-dev libtinfo5 cmake libffi-dev libssl-dev

# Initialize buildozer (first time only)
buildozer init

# Build APK (takes 30-60 minutes first time)
buildozer android debug

# APK output: bin/gyrocontroller-*.apk
```

### Step 16: Configure App

1. Open app
2. Tap ⚙️ (Settings)
3. Enter Robot IP: `192.168.1.100` (your ESP32 IP)
4. Control Port: `5000`
5. Telemetry Port: `5001`
6. Video Port: `5002` (optional)
7. Save settings

---

## Initial Testing

### Step 17: Power-On Checks

**Safety First:**
- Work on a non-conductive surface
- Have fire extinguisher nearby (LiPo safety)
- Be ready to disconnect battery quickly

**Power-On Sequence:**

1. **Visual Inspection**
   - Check all connections are secure
   - Verify no shorts (use multimeter)
   - Check battery is charged (11.1-12.6V)

2. **Initial Power-Up**
   - Connect battery
   - Turn on power switch
   - Observe LEDs on ESP32 modules
   - Listen for initialization sounds

3. **Serial Monitor Check**
   ```bash
   idf.py -p /dev/ttyUSB0 monitor
   ```
   - WiFi should connect within 10 seconds
   - IP address displayed
   - All sensors initialized

4. **Network Ping Test**
   ```bash
   ping <robot-ip-address>
   # Should see replies with <10ms latency
   ```

### Step 18: Component Testing

#### Test 1: IMU Readings

**With serial monitor open:**
- Tilt robot forward → pitch should increase
- Tilt backward → pitch should decrease
- Rotate left/right → yaw should change
- Level robot → pitch/roll near 0°

#### Test 2: Motor Test (NO LOAD)

**Safety: Lift robot off ground**

Send test commands via app or server:
```python
# In server test script
send_motor_command(left=50, right=50)  # Both forward slow
# Observe: both motors spin same direction
```

If one motor spins backward:
- Power off
- Swap that motor's wires on DRV8833

#### Test 3: Balance Response (HOLD ROBOT)

**Hold robot vertically:**
- Tilt forward slightly → motors should push forward
- Tilt backward → motors should push backward
- Response should be smooth, not jerky

If oscillates wildly:
- Reduce Kp gain in firmware
- Increase Kd gain

#### Test 4: Servo Movement

Using app servo controls:
- Move slider → camera should tilt
- Movement should be smooth
- Test full range (0-180°)

#### Test 5: Camera Stream

Run server:
```bash
python main.py
```
- Video window should appear
- Image should be clear (not blurry)
- Frame rate displayed (~15-30 FPS)

---

## Calibration

### Step 19: IMU Calibration

#### Accelerometer Calibration

```bash
# Place robot on perfectly level surface
# Run calibration routine (in firmware)
idf.py -p /dev/ttyUSB0 monitor
# Press 'c' to start calibration (if implemented)
# Or use factory calibration routine
```

#### Mahony Filter Tuning

Start with defaults:
- Kp = 2.0
- Ki = 0.01

**If IMU drifts over time:**
- Increase Ki slightly (0.05 - 0.1)

**If IMU is noisy/jittery:**
- Decrease Kp (1.5 - 1.8)
- But not too low or response becomes sluggish

### Step 20: PID Tuning for Balance

**Tuning Procedure (Ziegler-Nichols Method):**

1. **Start Conservative**
   ```
   Kp = 20
   Ki = 0
   Kd = 0
   ```

2. **Find Critical Gain (Kp)**
   - Gradually increase Kp
   - At some point, robot will oscillate continuously
   - Note this value as Kp_critical
   - Reduce Kp to 60% of Kp_critical

3. **Add Derivative (Kd)**
   - Start with Kd = Kp / 10
   - Increase until oscillations are damped
   - Typical range: 0.5 - 2.0

4. **Add Integral (Ki)**
   - Start with Ki = 0
   - If robot has steady-state tilt, add small Ki
   - Typical range: 0 - 0.5
   - Too much Ki causes overshoot

5. **Test Under Load**
   - Place small weight on robot
   - Verify still balances
   - Adjust if needed

**Example Tuned Values:**
```
Kp = 35.0  (aggressive response)
Ki = 0.2   (eliminates drift)
Kd = 1.5   (dampens oscillation)
```

### Step 21: Motor Calibration

#### Deadband Adjustment

Motors have a deadband where low PWM doesn't move them:

1. Test minimum PWM:
   ```cpp
   // In firmware, test incrementally
   motor_left.set_speed(10);  // Too low?
   motor_left.set_speed(20);  // Starts moving?
   ```

2. Set deadband threshold in code

3. Apply deadband compensation:
   ```cpp
   if (abs(speed) < DEADBAND) {
       speed = 0;
   } else {
       speed = speed + sign(speed) * DEADBAND;
   }
   ```

### Step 22: Distance Sensor Calibration

**HC-SR04 Calibration:**

1. Measure known distances (20cm, 50cm, 100cm)
2. Record sensor readings
3. Check for consistent offset
4. Apply correction in firmware if needed

**Typical accuracy:** ±1cm

### Step 23: Camera Calibration

**Focus and Exposure:**
- Adjust camera focus ring (if available)
- Set exposure in firmware config
- Test in various lighting conditions

**Servo Angle Mapping:**
- Determine useful angle range (typically 45-135°)
- Set software limits to prevent mechanical strain

---

## Final Assembly Checklist

### Mechanical
- [ ] All screws tightened
- [ ] Motors firmly mounted
- [ ] Wheels attached securely
- [ ] Battery secured with velcro
- [ ] No loose wires

### Electrical
- [ ] All connections soldered (no loose wires)
- [ ] Heat shrink applied
- [ ] No exposed metal (short circuit risk)
- [ ] Buck converters set to correct voltages
- [ ] Power switch functional

### Software
- [ ] Control firmware flashed and running
- [ ] Vision firmware flashed and running
- [ ] WiFi connects automatically
- [ ] Server runs without errors
- [ ] Mobile app connects successfully

### Calibration
- [ ] IMU calibrated
- [ ] PID tuned for balance
- [ ] Motors spinning correct directions
- [ ] Servo range tested
- [ ] Camera streaming

---

## Troubleshooting Build Issues

### Issue: ESP32 Won't Program

**Solutions:**
- Hold BOOT button while clicking upload
- Check USB cable (some are charge-only)
- Try different USB port
- Verify CH340/CP2102 drivers installed

### Issue: Motors Don't Spin

**Check:**
- DRV8833 powered (10.5V present)
- Motor connections secure
- PWM signals present (oscilloscope/multimeter)
- Motor driver not in thermal shutdown (too hot)

### Issue: IMU Values Wrong

**Check:**
- I2C connections (SDA/SCL not swapped)
- Pull-up resistors present (usually on MPU6050 board)
- Correct I2C address (0x68 or 0x69)
- Power stable (3.3V)

### Issue: Camera No Video

**Check:**
- ESP32-CAM powered (needs 5V, high current)
- Camera ribbon cable seated properly
- Firmware uploaded correctly (OTA or UART)
- Network connection established

### Issue: Robot Falls Over Immediately

**Check:**
- IMU orientation correct (align with robot axis)
- Motor directions correct (both push same way)
- PID gains not too aggressive
- Balance loop running (check serial monitor)

---

## Next Steps

After successful build:
1. Test in safe, open area
2. Tune PID gains for your specific build
3. Test autonomous features
4. Experiment with different modes
5. Customize and extend functionality

## Safety Reminders

- Always supervise operation
- LiPo batteries are fire hazards if damaged
- Use appropriate battery charger
- Don't leave battery connected when not in use
- Monitor motor/driver temperatures
- Keep firmware updated
- Have emergency stop ready

## Support

For build issues:
- Check documentation: `docs/`
- Review code comments
- Test components individually
- Use serial monitor for debugging
- GitHub issues for project-specific problems

Happy building! 🤖
7. [Mobile App Setup](#mobile-app-setup)
8. [Initial Testing](#initial-testing)
9. [Calibration](#calibration)

---

## Prerequisites

### Tools Required

#### Mechanical Tools
- Soldering iron and solder
- Wire strippers
- Small screwdrivers (Phillips and flat head)
- Hex key set
- Multimeter
- Heat shrink tubing and heat gun
- Helping hands/PCB holder
- Diagonal cutters

#### Software Tools
- Computer with USB ports
- USB to TTL serial adapter (or USB cables for ESP32)
- Text editor or IDE (VS Code recommended)
- Git for version control

### Parts List

#### Microcontrollers
- [ ] 1x ESP32 DevKit (main controller)
- [ ] 1x ESP32-CAM module with OV7670 camera
- [ ] 1x FPGA board (TBD based on requirements)

#### Sensors
- [ ] 1x MPU6050 6-axis IMU module
- [ ] 1x HC-SR04 ultrasonic distance sensor
- [ ] Connecting wires (Dupont cables)

#### Motors & Drivers
- [ ] 2x DC geared motors (12V, 150 RPM, 2kg·cm torque)
- [ ] 1x DRV8833 dual motor driver module
- [ ] 2x Wheels (compatible with motor shaft)

#### Servo
- [ ] 1x MG90S micro servo motor

#### Power System
- [ ] 1x LiPo battery 3S (11.1V, 2200mAh, 80C)
- [ ] 1x LiPo battery charger (with balance charging)
- [ ] 1x Buck converter 5V 6A
- [ ] 1x Buck converter adjustable (for 10.5V)
- [ ] 1x XT60 connector (battery connector)
- [ ] 1x Power switch
- [ ] Battery voltage alarm (optional but recommended)

#### Electronics
- [ ] Perfboard or custom PCB
- [ ] 22-24 AWG wire (red and black for power)
- [ ] 26-28 AWG wire (for signals)
- [ ] Pin headers (male and female)
- [ ] JST connectors (optional, for modular connections)
- [ ] Heat shrink tubing (various sizes)
- [ ] Electrical tape

#### Mechanical Parts
- [ ] 3D printed chassis (files in `hardware/model_design/`)
- [ ] M3 screws and nuts (various lengths: 8mm, 12mm, 16mm)
- [ ] M2.5 screws (for motor mounts)
- [ ] Standoffs (M3, 20mm and 40mm)
- [ ] Velcro straps or zip ties

---

## Hardware Assembly

### Step 1: 3D Printing

#### Print the Chassis Components

Navigate to `hardware/model_design/` and print the following STL files:

1. **GyroBot-Base.stl**
   - Material: PLA or PETG
   - Layer height: 0.2mm
   - Infill: 20-30%
   - Supports: Yes
   - Print time: ~4-6 hours

2. **GyroBot-MotorMount.stl** (print 2x)
   - Material: PLA or PETG
   - Layer height: 0.2mm
   - Infill: 30-40% (needs strength)
   - Supports: Minimal
   - Print time: ~1-2 hours each

3. **GyroBot-Parts.stl**
   - Contains: Servo mount, camera bracket, standoffs
   - Material: PLA
   - Layer height: 0.2mm
   - Infill: 20%
   - Print time: ~2-3 hours

#### Post-Processing
- Remove supports carefully
- Clean up rough edges with sandpaper
- Test fit all screw holes (drill out if tight)
- Dry fit motors in motor mounts

### Step 2: Motor Assembly

#### Mount Motors to Motor Mounts

1. Insert motor into motor mount bracket
2. Align screw holes
3. Secure with M2.5 screws (4 per motor)
4. Ensure motor shaft rotates freely
5. Attach wheels to motor shafts

#### Attach Motor Mounts to Base

1. Position motor mounts on base chassis
2. Align mounting holes
3. Use M3 x 16mm screws and nuts
4. Tighten firmly but don't strip plastic
5. Verify both wheels are level and aligned

### Step 3: Create the Three-Stack Structure

#### Base Stack (Layer 1)
- Chassis with motors attached
- Battery mounting area (secure with velcro)

#### Second Stack (Layer 2)
- Mount on standoffs (40mm M3 standoffs from base)
- Will hold main circuit board
- Buck converters mounted here

#### Third Stack (Layer 3)
- Mount on standoffs (20mm M3 standoffs from layer 2)
- Camera mount with servo
- FPGA mounting area

---

## Electronics Assembly

### Step 4: Power Distribution Board

Create the power distribution circuit on perfboard:

#### Power Distribution Schematic

```
Battery (11.1V) → Power Switch → Split
                                  ├── Buck #1 (5V 6A)
                                  │   ├→ ESP32
                                  │   ├→ ESP32-CAM
                                  │   ├→ HC-SR04
                                  │   └→ Servo
                                  │
                                  └── Buck #2 (10.5V)
                                      └→ DRV8833 Motor Driver
```

#### Assembly Steps

1. **Solder Buck Converters**
   - Buck #1: Set output to exactly 5.0V (measure with multimeter)
   - Buck #2: Set output to 10.5V
   - Solder input wires (thick, 22 AWG)
   - Solder output wires with polarity markings

2. **Create Power Rails**
   - Use perfboard to create 5V and GND rails
   - Solder thick wire for current capacity
   - Add capacitors (100µF) for noise filtering

3. **Wire Power Switch**
   - Connect battery positive to switch
   - Connect switch output to distribution point
   - Test continuity with multimeter

4. **Wire XT60 Battery Connector**
   - Solder to power input (before switch)
   - Verify polarity (red = positive, black = negative)
   - Use heat shrink on connections

### Step 5: Main Controller Wiring

#### ESP32 Connections

Create a wiring harness for the ESP32:

**Motor Driver (DRV8833)**
```
ESP32 GPIO → DRV8833
GPIO 12    → AIN1 (Motor A forward)
GPIO 13    → AIN2 (Motor A backward)
GPIO 14    → BIN1 (Motor B forward)
GPIO 27    → BIN2 (Motor B backward)
```

**IMU (MPU6050)**
```
ESP32 → MPU6050
3.3V  → VCC
GND   → GND
GPIO 21 → SDA
GPIO 22 → SCL
```

**Ultrasonic Sensor (HC-SR04)**
```
ESP32 → HC-SR04
5V    → VCC
GND   → GND
GPIO 18 → TRIG
GPIO 19 → ECHO
```

**Servo (MG90S)**
```
ESP32 → Servo
5V    → VCC (red wire)
GND   → GND (brown/black wire)
GPIO 25 → Signal (orange/yellow wire)
```

**ESP32-CAM Communication** (optional, if not standalone)
```
ESP32 → ESP32-CAM
GPIO 16 (RX2) → TX
GPIO 17 (TX2) → RX
GND → GND
```

#### Soldering Tips
- Tin all wires before connecting
- Use heat shrink on all connections
- Keep wire runs short and neat
- Label all wires with tape/markers
- Test continuity after each connection

### Step 6: Motor Driver Setup

#### DRV8833 Connections

1. **Power Input**
   - Connect VM to 10.5V from Buck #2
   - Connect GND to common ground
   - Add 100µF capacitor across VM and GND (close to driver)

2. **Motor Outputs**
   - Connect Motor A to AOUT1 and AOUT2
   - Connect Motor B to BOUT1 and BOUT2
   - Polarity doesn't matter (can reverse in code)

3. **Logic Power**
   - Connect VCC to 3.3V from ESP32
   - Connect GND to common ground

### Step 7: Camera Module Assembly

#### ESP32-CAM Setup

1. **Program Before Installation**
   - ESP32-CAM needs USB-TTL adapter for programming
   - Connect: GND-GND, 5V-5V, U0R-TX, U0T-RX
   - Connect GPIO 0 to GND for programming mode
   - Upload firmware
   - Remove GPIO 0 connection for normal operation

2. **Mount Camera and Servo**
   - Attach servo to camera mount (printed part)
   - Attach ESP32-CAM to servo arm
   - Connect servo signal wire
   - Secure with small screws

3. **Power Connection**
   - 5V from Buck #1 to ESP32-CAM 5V pin
   - Common ground

---

## Firmware Setup

### Step 8: Install Development Environment

#### Install ESP-IDF

```bash
# Clone ESP-IDF repository
cd ~
git clone -b v4.4.6 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32

# Add to shell profile (bash/zsh)
echo ". ~/esp-idf/export.sh" >> ~/.bashrc
source ~/.bashrc
```

#### Clone Project Repository

```bash
git clone https://github.com/Zeta-Chrome/gyro-bot.git
cd gyro-bot
```

### Step 9: Configure and Build Control Firmware

```bash
cd firmware/control

# Load ESP-IDF environment
source ../env.sh
# Or if not using env.sh:
# . ~/esp-idf/export.sh

# Configure project
idf.py menuconfig
```

#### Configuration Steps in menuconfig

1. **WiFi Configuration**
   - Navigate: `Component config` → `WiFi Station Configuration`
   - Set `WiFi SSID`: Your network name
   - Set `WiFi Password`: Your network password

2. **Network Configuration**
   - Set server IP address (your PC's IP)
   - Set UDP control port (default: 5000)
   - Set UDP telemetry port (default: 5001)

3. **IMU Configuration**
   - Set I2C pins (default: SDA=21, SCL=22)
   - Set sample rate (default: 100Hz)
   - Set Mahony filter gains (Kp=2.0, Ki=0.01)

4. **Motor Configuration**
   - Set PWM frequency (default: 20kHz)
   - Set motor GPIO pins
   - Set maximum PWM duty cycle

5. **PID Tuning**
   - Set initial PID values (Kp=20, Ki=0, Kd=0.5)
   - These will need tuning after assembly

6. Save and exit

#### Build and Flash

```bash
# Build firmware
idf.py build

# Identify ESP32 port
ls /dev/tty*  # Look for /dev/ttyUSB0 or similar

# Flash to ESP32
idf.py -p /dev/ttyUSB0 flash

# Monitor output (Ctrl+] to exit)
idf.py -p /dev/ttyUSB0 monitor
```

Verify in monitor output:
- WiFi connection successful
- IP address displayed
- IMU initialized
- Motor driver ready

### Step 10: Configure and Build Vision Firmware

```bash
cd ../vision

# Configure (similar WiFi setup)
idf.py menuconfig

# Set camera configuration
# - Frame size: QVGA (320x240) or VGA (640x480)
# - JPEG quality: 12 (lower = better quality, larger size)
# - Frame rate: 15-30 FPS

# Build and flash
idf.py build
idf.py -p /dev/ttyUSB1 flash  # Note: different port
idf.py -p /dev/ttyUSB1 monitor
```

Verify:
- Camera initialization successful
- TCP server started
- Streaming ready

---

## Server Setup

### Step 11: Python Environment

```bash
cd ../../server

# Create virtual environment
python3 -m venv venv

# Activate (Linux/Mac)
source venv/bin/activate
# Windows:
# venv\Scripts\activate

# Install dependencies
pip install --upgrade pip
pip install ultralytics opencv-python numpy torch matplotlib
```

### Step 12: Configure Server

Edit `config.py`:

```python
# Network Configuration
ROBOT_IP = "192.168.1.100"  # ESP32 IP from serial monitor
UDP_CONTROL_PORT = 5000
UDP_TELEMETRY_PORT = 5001
TCP_VIDEO_PORT = 5002

# Detection Configuration
YOLO_MODEL = "yolov8n.pt"  # Nano model (fastest)
CONFIDENCE_THRESHOLD = 0.5
IOU_THRESHOLD = 0.4

# Display Configuration
SHOW_VIDEO = True
SHOW_DETECTIONS = True
```

### Step 13: Download YOLO Model

```bash
# Model will auto-download on first run, or manual:
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

### Step 14: Test Server

```bash
python main.py
```

Expected output:
- Connecting to robot...
- Receiving telemetry
- Video stream active
- Press 'q' to quit

---

## Mobile App Setup

### Step 15: Install App on Android

#### Option A: Pre-built APK (Quick)

1. Transfer `app/bin/gyrocontroller-1.0-arm64-v8a-debug.apk` to Android device
2. Enable "Install from Unknown Sources" in Android settings
3. Open file manager and install APK
4. Open GyroController app

#### Option B: Build from Source

```bash
cd app/

# Install Buildozer
pip install buildozer

# Install dependencies (Ubuntu/Debian)
sudo apt update
sudo apt install -y git zip unzip openjdk-11-jdk python3-pip autoconf libtool pkg-config zlib1g-dev libncurses5-dev libncursesw5-dev libtinfo5 cmake libffi-dev libssl-dev

# Initialize buildozer (first time only)
buildozer init

# Build APK (takes 30-60 minutes first time)
buildozer android debug

# APK output: bin/gyrocontroller-*.apk
```

### Step 16: Configure App

1. Open app
2. Tap ⚙️ (Settings)
3. Enter Robot IP: `192.168.1.100` (your ESP32 IP)
4. Control Port: `5000`
5. Telemetry Port: `5001`
6. Video Port: `5002` (optional)
7. Save settings

---

## Initial Testing

### Step 17: Power-On Checks

**Safety First:**
- Work on a non-conductive surface
- Have fire extinguisher nearby (LiPo safety)
- Be ready to disconnect battery quickly

**Power-On Sequence:**

1. **Visual Inspection**
   - Check all connections are secure
   - Verify no shorts (use multimeter)
   - Check battery is charged (11.1-12.6V)

2. **Initial Power-Up**
   - Connect battery
   - Turn on power switch
   - Observe LEDs on ESP32 modules
   - Listen for initialization sounds

3. **Serial Monitor Check**
   ```bash
   idf.py -p /dev/ttyUSB0 monitor
   ```
   - WiFi should connect within 10 seconds
   - IP address displayed
   - All sensors initialized

4. **Network Ping Test**
   ```bash
   ping <robot-ip-address>
   # Should see replies with <10ms latency
   ```

### Step 18: Component Testing

#### Test 1: IMU Readings

**With serial monitor open:**
- Tilt robot forward → pitch should increase
- Tilt backward → pitch should decrease
- Rotate left/right → yaw should change
- Level robot → pitch/roll near 0°

#### Test 2: Motor Test (NO LOAD)

**Safety: Lift robot off ground**

Send test commands via app or server:
```python
# In server test script
send_motor_command(left=50, right=50)  # Both forward slow
# Observe: both motors spin same direction
```

If one motor spins backward:
- Power off
- Swap that motor's wires on DRV8833

#### Test 3: Balance Response (HOLD ROBOT)

**Hold robot vertically:**
- Tilt forward slightly → motors should push forward
- Tilt backward → motors should push backward
- Response should be smooth, not jerky

If oscillates wildly:
- Reduce Kp gain in firmware
- Increase Kd gain

#### Test 4: Servo Movement

Using app servo controls:
- Move slider → camera should tilt
- Movement should be smooth
- Test full range (0-180°)

#### Test 5: Camera Stream

Run server:
```bash
python main.py
```
- Video window should appear
- Image should be clear (not blurry)
- Frame rate displayed (~15-30 FPS)

---

## Calibration

### Step 19: IMU Calibration

#### Accelerometer Calibration

```bash
# Place robot on perfectly level surface
# Run calibration routine (in firmware)
idf.py -p /dev/ttyUSB0 monitor
# Press 'c' to start calibration (if implemented)
# Or use factory calibration routine
```

#### Mahony Filter Tuning

Start with defaults:
- Kp = 2.0
- Ki = 0.01

**If IMU drifts over time:**
- Increase Ki slightly (0.05 - 0.1)

**If IMU is noisy/jittery:**
- Decrease Kp (1.5 - 1.8)
- But not too low or response becomes sluggish

### Step 20: PID Tuning for Balance

**Tuning Procedure (Ziegler-Nichols Method):**

1. **Start Conservative**
   ```
   Kp = 20
   Ki = 0
   Kd = 0
   ```

2. **Find Critical Gain (Kp)**
   - Gradually increase Kp
   - At some point, robot will oscillate continuously
   - Note this value as Kp_critical
   - Reduce Kp to 60% of Kp_critical

3. **Add Derivative (Kd)**
   - Start with Kd = Kp / 10
   - Increase until oscillations are damped
   - Typical range: 0.5 - 2.0

4. **Add Integral (Ki)**
   - Start with Ki = 0
   - If robot has steady-state tilt, add small Ki
   - Typical range: 0 - 0.5
   - Too much Ki causes overshoot

5. **Test Under Load**
   - Place small weight on robot
   - Verify still balances
   - Adjust if needed

**Example Tuned Values:**
```
Kp = 35.0  (aggressive response)
Ki = 0.2   (eliminates drift)
Kd = 1.5   (dampens oscillation)
```

### Step 21: Motor Calibration

#### Deadband Adjustment

Motors have a deadband where low PWM doesn't move them:

1. Test minimum PWM:
   ```cpp
   // In firmware, test incrementally
   motor_left.set_speed(10);  // Too low?
   motor_left.set_speed(20);  // Starts moving?
   ```

2. Set deadband threshold in code

3. Apply deadband compensation:
   ```cpp
   if (abs(speed) < DEADBAND) {
       speed = 0;
   } else {
       speed = speed + sign(speed) * DEADBAND;
   }
   ```

### Step 22: Distance Sensor Calibration

**HC-SR04 Calibration:**

1. Measure known distances (20cm, 50cm, 100cm)
2. Record sensor readings
3. Check for consistent offset
4. Apply correction in firmware if needed

**Typical accuracy:** ±1cm

### Step 23: Camera Calibration

**Focus and Exposure:**
- Adjust camera focus ring (if available)
- Set exposure in firmware config
- Test in various lighting conditions

**Servo Angle Mapping:**
- Determine useful angle range (typically 45-135°)
- Set software limits to prevent mechanical strain

---

## Final Assembly Checklist

### Mechanical
- [ ] All screws tightened
- [ ] Motors firmly mounted
- [ ] Wheels attached securely
- [ ] Battery secured with velcro
- [ ] No loose wires

### Electrical
- [ ] All connections soldered (no loose wires)
- [ ] Heat shrink applied
- [ ] No exposed metal (short circuit risk)
- [ ] Buck converters set to correct voltages
- [ ] Power switch functional

### Software
- [ ] Control firmware flashed and running
- [ ] Vision firmware flashed and running
- [ ] WiFi connects automatically
- [ ] Server runs without errors
- [ ] Mobile app connects successfully

### Calibration
- [ ] IMU calibrated
- [ ] PID tuned for balance
- [ ] Motors spinning correct directions
- [ ] Servo range tested
- [ ] Camera streaming

---

## Troubleshooting Build Issues

### Issue: ESP32 Won't Program

**Solutions:**
- Hold BOOT button while clicking upload
- Check USB cable (some are charge-only)
- Try different USB port
- Verify CH340/CP2102 drivers installed

### Issue: Motors Don't Spin

**Check:**
- DRV8833 powered (10.5V present)
- Motor connections secure
- PWM signals present (oscilloscope/multimeter)
- Motor driver not in thermal shutdown (too hot)

### Issue: IMU Values Wrong

**Check:**
- I2C connections (SDA/SCL not swapped)
- Pull-up resistors present (usually on MPU6050 board)
- Correct I2C address (0x68 or 0x69)
- Power stable (3.3V)

### Issue: Camera No Video

**Check:**
- ESP32-CAM powered (needs 5V, high current)
- Camera ribbon cable seated properly
- Firmware uploaded correctly (OTA or UART)
- Network connection established

### Issue: Robot Falls Over Immediately

**Check:**
- IMU orientation correct (align with robot axis)
- Motor directions correct (both push same way)
- PID gains not too aggressive
- Balance loop running (check serial monitor)

---

## Next Steps

After successful build:
1. Test in safe, open area
2. Tune PID gains for your specific build
3. Test autonomous features
4. Experiment with different modes
5. Customize and extend functionality

## Safety Reminders

- Always supervise operation
- LiPo batteries are fire hazards if damaged
- Use appropriate battery charger
- Don't leave battery connected when not in use
- Monitor motor/driver temperatures
- Keep firmware updated
- Have emergency stop ready

## Support

For build issues:
- Check documentation: `docs/`
- Review code comments
- Test components individually
- Use serial monitor for debugging
- GitHub issues for project-specific problems
