"""
Main autonomous navigation system
"""
import time
import numpy as np

from sensor_receiver import SensorReceiver
from sensor_fusion import SensorFusion
from object_detector import ObjectDetector
from path_planner import PathPlanner
from display import Display
import config


class AutonomousNavigator:
    def __init__(self):
        print("=" * 60)
        print("Autonomous Navigation System")
        print("=" * 60)
        
        # Initialize components
        print("[INIT] Starting sensor receiver...")
        self.receiver = SensorReceiver()
        
        print("[INIT] Initializing sensor fusion...")
        self.fusion = SensorFusion()
        
        print("[INIT] Loading object detector...")
        self.detector = ObjectDetector()
        
        print("[INIT] Initializing path planner...")
        self.planner = PathPlanner()
        
        print("[INIT] Setting up display...")
        self.display = Display()
        
        self.running = False
        self.loop_count = 0
        
    def run(self):
        """Main navigation loop"""
        print("\n[SYSTEM] Starting navigation system...")
        print("[SYSTEM] Press 'q' or ESC to quit\n")
        
        # Start receiver threads
        self.receiver.start()
        self.running = True
        
        # Wait for connections
        print("[SYSTEM] Waiting for ESP32 connection...")
        print("[SYSTEM] Checking for IMU data...")
        
        # Wait for IMU data first
        for i in range(20):
            imu_samples = self.receiver.get_latest_imu(n=1)
            if imu_samples:
                print(f"[SYSTEM] ✓ IMU data received!")
                break
            time.sleep(0.5)
            if i % 4 == 0:
                print(f"[SYSTEM] Still waiting for IMU... ({i//2}s)")
        
        # Wait for camera
        print("[SYSTEM] Waiting for camera connection...")
        for i in range(20):
            if self.receiver.camera_connected:
                print(f"[SYSTEM] ✓ Camera connected!")
                break
            time.sleep(0.5)
            if i % 4 == 0:
                print(f"[SYSTEM] Still waiting for camera... ({i//2}s)")
        
        if not self.receiver.camera_connected:
            print("[SYSTEM] ⚠ Camera not connected, but continuing anyway...")
            print("[SYSTEM] Make sure ESP32 is sending to port 9006")
        
        try:
            while self.running:
                start_time = time.time()
                
                # 1. Get sensor data
                imu_samples = self.receiver.get_latest_imu(n=10)
                us_samples = self.receiver.get_latest_ultrasound(n=5)
                frame = self.receiver.get_latest_frame()
                
                # 2. Process sensors
                self.fusion.update_imu(imu_samples)
                distance = self.fusion.update_ultrasound(us_samples)
                orientation = self.fusion.get_orientation()
                
                # 3. Run object detection
                detections = []
                annotated_frame = frame
                
                if frame is not None:
                    if self.loop_count == 1:
                        print(f"[SYSTEM] ✓ First frame received! Shape: {frame.shape}")
                    
                    detections = self.detector.detect(frame)
                    annotated_frame = self.detector.get_annotated_frame()
                    obstacles = self.detector.get_obstacles_in_path(frame.shape[1])
                else:
                    obstacles = []
                    if self.loop_count % 100 == 0:
                        print(f"[SYSTEM] Still no camera frames (loop {self.loop_count})")
                
                
                # 4. Plan path
                control = self.planner.plan(distance, obstacles, orientation)
                
                # 5. Send control commands
                self.receiver.send_control(
                    control['magnitude'],
                    control['angle'],
                    control['servo_angle']
                )
                
                # 6. Update display
                sensor_data = {
                    'distance': distance,
                    'orientation': orientation
                }
                
                self.display.update(annotated_frame, sensor_data, control, detections)
                
                # 7. Check for quit
                if self.display.check_quit():
                    print("\n[SYSTEM] User requested quit")
                    break
                    
                # 8. Status updates
                self.loop_count += 1
                if self.loop_count % 30 == 0:
                    print(f"[STATUS] Distance: {distance:.1f}cm | "
                          f"Yaw: {orientation['yaw']:.1f}° | "
                          f"State: {control['state']} | "
                          f"Detections: {len(detections)}")
                
                # Control loop rate
                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0 / 30) - elapsed)  # 30Hz target
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n[SYSTEM] Interrupted by user")
        except Exception as e:
            print(f"\n[ERROR] System error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Clean shutdown"""
        print("\n[SYSTEM] Shutting down...")
        self.running = False
        
        # Stop robot
        print("[SYSTEM] Stopping robot...")
        self.receiver.send_control(0, 0, config.SERVO_CENTER)
        time.sleep(0.5)
        
        # Stop receiver
        print("[SYSTEM] Stopping receivers...")
        self.receiver.stop()
        
        # Close display
        print("[SYSTEM] Closing display...")
        self.display.cleanup()
        
        print("[SYSTEM] Shutdown complete\n")


def main():
    navigator = AutonomousNavigator()
    navigator.run()


if __name__ == "__main__":
    main()
