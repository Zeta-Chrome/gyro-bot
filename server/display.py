"""
Visualization and display
"""
import cv2
import numpy as np
import time
import config


class Display:
    def __init__(self):
        self.window_name = "Autonomous Navigation"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, config.DISPLAY_WIDTH, config.DISPLAY_HEIGHT)
        
        self.fps = 0
        self.frame_count = 0
        self.last_fps_update = time.time()
        
    def create_info_panel(self, sensor_data, control_data):
        """Create information panel"""
        panel = np.zeros((200, 400, 3), dtype=np.uint8)
        
        y_offset = 25
        
        # Title
        cv2.putText(panel, "ROBOT STATUS", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        y_offset += 30
        
        # State
        state = control_data.get('state', 'UNKNOWN')
        state_color = {
            'EXPLORING': (0, 255, 0),
            'AVOIDING': (0, 165, 255),
            'CAUTIOUS': (0, 255, 255),
            'STOPPED': (0, 0, 255)
        }.get(state, (255, 255, 255))
        
        cv2.putText(panel, f"State: {state}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, state_color, 2)
        y_offset += 25
        
        # Reason
        reason = control_data.get('reason', 'N/A')
        cv2.putText(panel, f"Reason: {reason}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 25
        
        # Distance
        distance = sensor_data.get('distance', 0)
        dist_color = (0, 255, 0) if distance > config.SLOW_DISTANCE else \
                     (0, 255, 255) if distance > config.SAFE_DISTANCE else (0, 0, 255)
        cv2.putText(panel, f"Distance: {distance:.1f} cm", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, dist_color, 1)
        y_offset += 25
        
        # Orientation
        orient = sensor_data.get('orientation', {})
        cv2.putText(panel, f"Pitch: {orient.get('pitch', 0):.1f}deg", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 20
        cv2.putText(panel, f"Roll: {orient.get('roll', 0):.1f}deg", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 20
        cv2.putText(panel, f"Yaw: {orient.get('yaw', 0):.1f}deg", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 25
        
        # Control output
        cv2.putText(panel, f"Speed: {control_data.get('magnitude', 0):.2f}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
        y_offset += 20
        cv2.putText(panel, f"Turn: {control_data.get('angle', 0):.1f}deg", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
        
        return panel
        
    def create_minimap(self, distance, orientation):
        """Create top-down minimap view"""
        size = 300
        minimap = np.zeros((size, size, 3), dtype=np.uint8)
        center = size // 2
        
        # Draw grid
        for i in range(0, size, 30):
            cv2.line(minimap, (i, 0), (i, size), (30, 30, 30), 1)
            cv2.line(minimap, (0, i), (size, i), (30, 30, 30), 1)
            
        # Draw robot (center)
        yaw = orientation.get('yaw', 0)
        robot_size = 20
        
        # Robot body
        cv2.circle(minimap, (center, center), robot_size, (0, 255, 0), -1)
        
        # Direction indicator
        end_x = int(center + robot_size * 1.5 * np.sin(np.radians(yaw)))
        end_y = int(center - robot_size * 1.5 * np.cos(np.radians(yaw)))
        cv2.line(minimap, (center, center), (end_x, end_y), (255, 255, 0), 3)
        
        # Draw obstacle distance
        if distance < config.ULTRASOUND_MAX_RANGE:
            scale = 200 / config.ULTRASOUND_MAX_RANGE  # Map to pixels
            obs_dist = int(distance * scale)
            obs_x = int(center + obs_dist * np.sin(np.radians(yaw)))
            obs_y = int(center - obs_dist * np.cos(np.radians(yaw)))
            
            if 0 <= obs_x < size and 0 <= obs_y < size:
                cv2.circle(minimap, (obs_x, obs_y), 10, (0, 0, 255), -1)
                cv2.line(minimap, (center, center), (obs_x, obs_y), (0, 0, 255), 2)
                
        # Labels
        cv2.putText(minimap, "N", (center - 8, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(minimap, "MINIMAP", (10, size - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
        
        return minimap
        
    def update(self, frame, sensor_data, control_data, detections):
        """Update display with all information"""
        # Update FPS
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_update > 1.0:
            self.fps = self.frame_count / (current_time - self.last_fps_update)
            self.frame_count = 0
            self.last_fps_update = current_time
            
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "Waiting for camera...", (150, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        else:
            # Add FPS and detection count to the actual frame
            cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Detections: {len(detections)}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                       
        # Create info panel
        info_panel = self.create_info_panel(sensor_data, control_data)
        
        # Create minimap
        minimap = self.create_minimap(sensor_data.get('distance', 0), 
                                      sensor_data.get('orientation', {}))
        
        # Resize frame to standard size for display
        frame_h = 480
        frame_w = 640
        if frame.shape[0] != frame_h or frame.shape[1] != frame_w:
            frame_resized = cv2.resize(frame, (frame_w, frame_h))
        else:
            frame_resized = frame
        
        # Resize panels to match frame width
        info_resized = cv2.resize(info_panel, (frame_w // 2, 200))
        minimap_resized = cv2.resize(minimap, (frame_w // 2, 200))
        bottom_panel = np.hstack([info_resized, minimap_resized])
        
        # Final layout - stack vertically
        combined = np.vstack([frame_resized, bottom_panel])
        
        cv2.imshow(self.window_name, combined)
        
    def check_quit(self):
        """Check if user wants to quit"""
        key = cv2.waitKey(1) & 0xFF
        return key == ord('q') or key == 27
        
    def cleanup(self):
        """Close windows"""
        cv2.destroyAllWindows()
