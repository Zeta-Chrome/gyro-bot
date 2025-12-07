"""
Sensor fusion for IMU and ultrasound data filtering
"""
import numpy as np
from scipy.signal import butter, filtfilt
from collections import deque
import time
import config


class SensorFusion:
    def __init__(self):
        # IMU state
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        
        # Complementary filter parameters
        self.alpha = 0.98
        self.dt = 1.0 / config.IMU_SAMPLE_RATE
        
        # Ultrasound filtering
        self.distance_history = deque(maxlen=10)
        self.last_valid_distance = None
        self.last_update_time = time.time()
        
        # Low-pass filter for ultrasound
        self.us_filtered = None
        
    def update_imu(self, imu_samples):
        """Process IMU data and calculate orientation"""
        if not imu_samples:
            return
            
        # Use latest sample
        sample = imu_samples[-1]
        ax, ay, az = sample['ax'], sample['ay'], sample['az']
        gx, gy, gz = sample['gx'], sample['gy'], sample['gz']
        
        # Calculate pitch and roll from accelerometer
        pitch_acc = np.degrees(np.arctan2(ax, np.sqrt(ay**2 + az**2)))
        roll_acc = np.degrees(np.arctan2(ay, np.sqrt(ax**2 + az**2)))
        
        # Integrate gyroscope
        pitch_gyro = self.pitch + gx * self.dt
        roll_gyro = self.roll + gy * self.dt
        yaw_gyro = self.yaw + gz * self.dt
        
        # Complementary filter
        self.pitch = self.alpha * pitch_gyro + (1 - self.alpha) * pitch_acc
        self.roll = self.alpha * roll_gyro + (1 - self.alpha) * roll_acc
        self.yaw = yaw_gyro  # No complementary for yaw (no magnetometer)
        
        # Normalize yaw to [-180, 180]
        self.yaw = ((self.yaw + 180) % 360) - 180
        
    def update_ultrasound(self, us_samples):
        """Filter ultrasound data to remove spurious readings"""
        if not us_samples:
            return self._handle_timeout()
            
        # Get latest reading
        sample = us_samples[-1]
        distance = sample['distance']
        current_time = time.time()
        
        # Check if reading is valid
        if config.ULTRASOUND_MIN_RANGE <= distance <= config.ULTRASOUND_MAX_RANGE:
            self.distance_history.append(distance)
            self.last_valid_distance = distance
            self.last_update_time = current_time
            
            # Apply median filter
            if len(self.distance_history) >= 3:
                self.us_filtered = np.median(list(self.distance_history))
            else:
                self.us_filtered = distance
        else:
            # Invalid reading - use filtered value or predict
            return self._handle_timeout()
            
        return self.us_filtered
        
    def _handle_timeout(self):
        """Handle missing or invalid ultrasound data"""
        current_time = time.time()
        
        if self.last_valid_distance is None:
            return config.ULTRASOUND_MAX_RANGE
            
        # Check timeout
        if current_time - self.last_update_time > config.ULTRASOUND_TIMEOUT:
            # Assume obstacle is still there (conservative)
            return self.last_valid_distance if self.last_valid_distance else config.ULTRASOUND_MAX_RANGE
            
        return self.us_filtered if self.us_filtered else self.last_valid_distance
        
    def get_orientation(self):
        """Get current orientation (pitch, roll, yaw in degrees)"""
        return {
            'pitch': self.pitch,
            'roll': self.roll,
            'yaw': self.yaw
        }
        
    def get_distance(self):
        """Get filtered distance reading"""
        return self.us_filtered if self.us_filtered else config.ULTRASOUND_MAX_RANGE
        
    def is_level(self, threshold=15.0):
        """Check if robot is relatively level"""
        return abs(self.pitch) < threshold and abs(self.roll) < threshold
