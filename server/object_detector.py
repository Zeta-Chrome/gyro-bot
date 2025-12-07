"""
YOLO-based object detection for navigation
"""
import cv2
import numpy as np
from ultralytics import YOLO
import config


class ObjectDetector:
    def __init__(self):
        print("[DETECTOR] Loading YOLO model...")
        self.model = YOLO(config.YOLO_MODEL)
        print("[DETECTOR] Model loaded")
        
        # Detection results
        self.detections = []
        self.annotated_frame = None
        
    def detect(self, frame):
        """Run object detection on frame"""
        if frame is None:
            return []
            
        # Run inference
        results = self.model(frame, conf=config.DETECTION_CONFIDENCE, verbose=False)
        
        self.detections = []
        self.annotated_frame = frame.copy()
        
        if len(results) > 0:
            result = results[0]
            
            # Process detections
            if result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                confidences = result.boxes.conf.cpu().numpy()
                class_ids = result.boxes.cls.cpu().numpy().astype(int)
                
                for box, conf, cls_id in zip(boxes, confidences, class_ids):
                    x1, y1, x2, y2 = box
                    
                    detection = {
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'confidence': float(conf),
                        'class_id': int(cls_id),
                        'class_name': self.model.names[cls_id],
                        'center_x': int((x1 + x2) / 2),
                        'center_y': int((y1 + y2) / 2),
                        'width': int(x2 - x1),
                        'height': int(y2 - y1)
                    }
                    
                    self.detections.append(detection)
                    
                    # Draw on frame
                    color = self._get_color(cls_id)
                    cv2.rectangle(self.annotated_frame, 
                                (int(x1), int(y1)), (int(x2), int(y2)), 
                                color, 2)
                    
                    # Label
                    label = f"{detection['class_name']} {conf:.2f}"
                    cv2.putText(self.annotated_frame, label, 
                              (int(x1), int(y1) - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                              
        return self.detections
        
    def get_annotated_frame(self):
        """Get frame with detection boxes drawn"""
        return self.annotated_frame
        
    def get_obstacles_in_path(self, frame_width, threshold_y=0.7):
        """
        Get obstacles in the robot's path (bottom portion of frame)
        
        Args:
            frame_width: Width of the frame
            threshold_y: Consider obstacles in bottom threshold_y of frame
        """
        obstacles = []
        
        for det in self.detections:
            bbox = det['bbox']
            center_y = det['center_y']
            
            # Check if obstacle is in bottom portion (closer to robot)
            if center_y > frame_width * threshold_y:
                # Calculate angular position relative to center
                frame_center = frame_width / 2
                angle_offset = (det['center_x'] - frame_center) / frame_center * 45  # +/- 45 degrees
                
                obstacles.append({
                    'class': det['class_name'],
                    'angle': angle_offset,
                    'size': det['width'] * det['height'],
                    'confidence': det['confidence']
                })
                
        return obstacles
        
    def _get_color(self, class_id):
        """Get consistent color for class"""
        np.random.seed(class_id)
        return tuple(map(int, np.random.randint(0, 255, 3)))
