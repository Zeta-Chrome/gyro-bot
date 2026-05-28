"""
Main robot server application
Combines object detection, path mapping, and network communication
"""
import time
import threading
import cv2
import numpy as np
from torch import true_divide
from sensor_receiver import SensorReceiver
from object_detector import ObjectDetector
from gui import RobotGUI
import config


class RobotServer:
    def __init__(self):
        print("=" * 60)
        print("🤖 ROBOT CONTROL SERVER")
        print("=" * 60)
        
        # Components
        self.receiver = SensorReceiver()
        self.detector = ObjectDetector()
        self.gui = RobotGUI("Robot Control Server")
        
        # State
        self.running = False
        self.last_frame_time = 0
        self.frame_interval = 1.0 / 10.0  # 10 FPS processing
        
        # Last control data (for path estimation)
        self.last_magnitude = 0.0
        self.last_angle = 0.0
        
    def start(self):
        """Start all components"""
        print("\n[SERVER] Starting all components...")
        
        # Start receiver
        self.receiver.start()
        time.sleep(0.5)
        
        # Start processing thread
        self.running = True
        self.process_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.process_thread.start()
        
        # Start GUI update thread
        self.gui_thread = threading.Thread(target=self._gui_update_loop, daemon=True)
        self.gui_thread.start()
        
        print("[SERVER] ✓ All components started")
        print("\n[SERVER] Waiting for ESP32 and mobile app to connect...")
        print(f"[SERVER] ESP32 should send to ports {config.TCP_CAMERA_PORT} (camera), {config.UDP_IMU_PORT} (IMU)")
        print(f"[SERVER] Mobile app should connect to port {config.TCP_OUTPUT_PORT} (processed images)")
        print(f"[SERVER] Mobile app sends mode to port {config.UDP_MODE_PORT}")
        print()
        
        # Run GUI (blocking)
        self.gui.run()
        
        # Cleanup
        self.stop()
    
    def stop(self):
        """Stop all components"""
        print("\n[SERVER] Stopping...")
        self.running = False
        self.receiver.stop()
        print("[SERVER] ✓ Stopped")
    
    def _process_loop(self):
        """Main processing loop"""
        while self.running:
            try:
                now = time.time()
                
                # Rate limit processing
                if now - self.last_frame_time < self.frame_interval:
                    time.sleep(0.01)
                    continue
                
                self.last_frame_time = now
                
                # Get latest data
                frame = self.receiver.get_latest_frame()
                imu_samples = self.receiver.get_latest_imu(10)
                us_samples = self.receiver.get_latest_ultrasound(5)
                mode = self.receiver.get_current_mode()
                
                # Process based on mode
                if mode == "Object detection":
                    self._process_object_detection(frame)
                else:
                    # Unknown mode, try object detection
                    self._process_object_detection(frame)
                
            except Exception as e:
                print(f"[PROCESS] Error: {e}")
                time.sleep(0.1)
    
    def _process_object_detection(self, frame):
        """Process object detection mode"""
        if frame is None:
            return
        
        try:
            # Detect objects
            annotated_frame, detections = self.detector.detect(frame)
            
            # Create summary image
            output_frame = self.detector.create_summary_image(annotated_frame, detections)
            
            # Update GUI (will be picked up by GUI thread)
            self.last_detection_frame = output_frame
            
        except Exception as e:
            print(f"[DETECTION] Error: {e}")
    
    def _gui_update_loop(self):
        """Update GUI display"""
        self.last_detection_frame = None
        
        while self.running:
            try:
                # Update connection status
                esp32_connected = self.receiver.esp32_ip is not None
                camera_connected = self.receiver.is_camera_connected()
                app_connected = True 

                self.gui.update_connections(
                    esp32=esp32_connected,
                    camera=camera_connected,
                    app=app_connected
                )
                
                # Update status message
                mode = self.receiver.get_current_mode()
                if esp32_connected and camera_connected and app_connected:
                    status = f"🟢 All Systems Active | Processing at {len(self.detector.fps_history):.0f} FPS"
                elif esp32_connected:
                    status = "🟡 ESP32 Connected | Waiting for mobile app..."
                else:
                    status = "⚫ Waiting for ESP32 connection..."
                
                self.gui.update_status(status, mode)
                
                # Update detection display
                if hasattr(self, 'last_detection_frame') and self.last_detection_frame is not None:
                    self.gui.update_detection_display(self.last_detection_frame)
                    
                    # Update detection stats
                    stats = self.detector.get_stats()
                    stats_text = (
                        f"FPS: {stats['avg_fps']:.1f} | "
                        f"Total Detections: {stats['total_detections']} | "
                        f"Unique Objects: {stats['unique_classes']}\n"
                        f"Top Objects: {', '.join([f'{k}({v})' for k, v in list(stats['object_counts'].items())[:5]])}"
                    )
                    self.gui.update_detection_stats(stats_text)
                
                # Update GUI
                self.gui.update()
                
                time.sleep(0.033)  # ~30 FPS GUI update
                
            except Exception as e:
                if self.running:
                    print(f"[GUI-UPDATE] Error: {e}")
                time.sleep(0.1)


def main():
    """Main entry point"""
    try:
        server = RobotServer()
        server.start()
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user")
    except Exception as e:
        print(f"[MAIN] Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
