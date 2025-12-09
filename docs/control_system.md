# Control System

## Self-Balancing Algorithm

The robot maintains balance using a classic control theory approach with PID (Proportional-Integral-Derivative) feedback control.

### Balance Control Loop

#### Sensor Input: MPU6050 IMU
The MPU6050 provides two types of data:
- **Accelerometer**: Measures gravitational acceleration to determine tilt angle
- **Gyroscope**: Measures angular velocity (rate of rotation)

#### Mahony Filter
Raw sensor data is noisy and subject to drift. The Mahony filter is an efficient orientation estimation algorithm that fuses gyroscope and accelerometer data using feedback correction.

**Algorithm**:
```
// Error correction from accelerometer
error = accel_measured × gravity_reference

// Integral feedback (for bias compensation)
integral_feedback += Ki × error × dt

// Proportional + Integral correction
corrected_gyro = gyro_measured + Kp × error + integral_feedback

// Integrate corrected gyroscope
quaternion = integrate(quaternion, corrected_gyro, dt)

// Extract pitch angle from quaternion
pitch_angle = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1² + q2²))
```

**Parameters**:
- `Kp ≈ 2.0`: Proportional gain (correction strength)
- `Ki ≈ 0.01`: Integral gain (gyro bias compensation)
- Uses quaternion representation (avoids gimbal lock)

**Advantages over complementary filter**:
- Better dynamic response during rapid movements
- Automatic gyroscope bias correction
- More accurate during aggressive maneuvers
- Quaternion math prevents singularities

### PID Controller

The PID controller calculates motor correction based on pitch angle error:

```
error = target_angle - current_pitch
P_term = Kp × error
I_term = Ki × ∫error dt
D_term = Kd × (d(error)/dt)
motor_output = P_term + I_term + D_term
```

#### Tuning Parameters
- **Kp (Proportional)**: Immediate response to current error
  - Higher Kp: Faster response, but can cause oscillation
  - Too low: Slow response, robot falls
  
- **Ki (Integral)**: Corrects accumulated steady-state error
  - Eliminates drift over time
  - Too high: Causes overshoot and instability
  
- **Kd (Derivative)**: Dampens rapid changes
  - Predicts future error based on rate of change
  - Reduces oscillation and overshoot
  - Too high: Amplifies sensor noise

#### Control Loop Execution
1. Read IMU data (gyro + accel)
2. Apply complementary filter → pitch angle
3. Calculate error from target (usually 0° = vertical)
4. Compute PID terms
5. Apply limits and deadband
6. Set motor speeds via PWM

**Loop Frequency**: 100-200 Hz (10-5ms cycle time) for responsive balance control

## Motor Control

### Differential Drive

Two motors enable:
- **Forward/Backward**: Both motors same direction, same speed
- **Turning**: Motors at different speeds or opposite directions
- **Rotation in Place**: Motors opposite directions, same speed

### Motor Mapping

From control inputs to motor commands:

```
// Balance correction
balance_correction = PID_output

// User command (from joystick or autonomous)
forward_speed = user_forward
turn_rate = user_turn

// Combine
left_motor = balance_correction + forward_speed - turn_rate
right_motor = balance_correction + forward_speed + turn_rate

// Clamp to limits [-255, 255]
```

### PWM Control via DRV8833

The motor driver receives:
- **Speed**: PWM duty cycle (0-255)
- **Direction**: IN1/IN2 pin states per motor

Example for Motor A:
- Forward: IN1=PWM, IN2=LOW
- Backward: IN1=LOW, IN2=PWM
- Brake: IN1=HIGH, IN2=HIGH
- Coast: IN1=LOW, IN2=LOW

## Navigation Control

### Autonomous Mode

When running autonomous algorithms (either on server or FPGA), the system uses:

#### Path Planning
- Input: Object detection results + distance sensor data
- Output: Target waypoints and desired heading
- Algorithm: Combines:
  - Obstacle avoidance vectors
  - Goal-seeking behavior
  - Smooth path interpolation

#### Control Commands
The path planner generates high-level commands:
- `target_velocity`: Desired forward speed
- `target_heading`: Desired orientation
- `servo_angle`: Camera angle for looking ahead

These are converted to motor commands that work in harmony with the balance controller.

### Manual Control (Joystick)

The mobile app sends joystick positions:
- **Y-axis**: Forward/backward velocity
- **X-axis**: Turn rate (differential speed)
- **Servo buttons**: Camera tilt up/down

Commands are translated to motor speeds while respecting balance requirements.

## Sensor Fusion

### Purpose
Combine multiple sensors for robust state estimation:
- **IMU**: Orientation and angular velocity
- **Ultrasonic**: Obstacle distance
- **Camera**: Visual features and movement

### Integration Strategy

#### For Self-Balancing
- Primary: IMU pitch angle (complementary filtered)
- Secondary: Motor encoder feedback (if available)
- Validation: Ensure motor commands match expected tilt correction

#### For Navigation
- **Visual Odometry**: Camera tracks feature points to estimate movement
- **IMU Integration**: Gyroscope provides rotation rate
- **Distance Validation**: Ultrasonic confirms object distances from vision
- **Fusion**: Extended Kalman Filter or particle filter combines all inputs

This multi-sensor approach compensates for:
- Camera lack of depth information (single lens)
- IMU drift over time
- Ultrasonic sensor noise and limited field of view

## Servo Control

### Camera Positioning

The MG90 servo adjusts camera pitch for:
- **Level View**: 90° (horizontal) for normal operation
- **Look Ahead**: 60-70° for detecting ground obstacles
- **Look Up**: 110-120° for overhead obstacle detection

### Control Modes

#### Manual Mode
User controls servo angle via app buttons/sliders

#### Autonomous Mode
Servo angle determined by:
- Current task (exploring vs. navigating)
- Detected obstacles (look down if close obstacles)
- Velocity (look further ahead at higher speeds)

### Smooth Transitions
Rather than instant jumps, servo moves smoothly:
```
current_angle += (target_angle - current_angle) × smoothing_factor
```
This prevents jerky camera movement and reduces mechanical stress.

## Safety Features

### Software Safety
- Motor PWM limits (prevent over-speed)
- Battery voltage monitoring (cutoff at low voltage)
- Tilt angle limits (emergency stop if tilt too extreme)
- Watchdog timers (restart if tasks hang)
- Communication timeout (stop if no commands received)

### Control Limits
- Maximum tilt correction speed (prevent violent movements)
- Acceleration limits (smooth speed changes)
- Turn rate limits (prevent tipping during sharp turns)
