"""
High-performance object detection using YOLOv8
"""
import cv2
import numpy as np
from collections import defaultdict, deque
import time
from ultralytics import YOLO
import config


class ObjectDetector:
    def __init__(self):
        print("[DETECTOR] Loading YOLOv8 model...")
        # Use YOLOv8s (small) for better accuracy than nano
        self.model = YOLO('yolov8s.pt')
        self.model.fuse()  # Optimize model
        
        # Detection history
        self.object_counts = defaultdict(int)
        self.object_history = deque(maxlen=100)
        self.total_detections = 0
        
        # Performance tracking
        self.fps_history = deque(maxlen=30)
        self.last_time = time.time()
        
        print("[DETECTOR] Model loaded successfully")
        
    def detect(self, frame):
        """
        Perform object detection on frame
        Returns: annotated frame, detections list
        """
        start_time = time.time()
        
        # Frame comes in as RGB, convert to BGR for YOLO
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Run inference with lower confidence for better detection
        results = self.model.predict(
            frame_bgr,
            conf=0.25,  # Lower confidence threshold
            iou=0.45,   # NMS threshold
            verbose=False,
            device='cpu',
            half=False,  # No FP16 on CPU
            imgsz=320    # Match input size
        )[0]
        
        # Process detections
        detections = []
        annotated_frame = frame_bgr.copy()
        
        if results.boxes is not None and len(results.boxes) > 0:
            boxes = results.boxes.xyxy.cpu().numpy()
            scores = results.boxes.conf.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy().astype(int)
            
            for box, score, cls in zip(boxes, scores, classes):
                x1, y1, x2, y2 = box.astype(int)
                class_name = results.names[cls]
                
                # Update counts
                self.object_counts[class_name] += 1
                self.total_detections += 1
                
                detection_info = {
                    'class': class_name,
                    'confidence': float(score),
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'timestamp': time.time()
                }
                detections.append(detection_info)
                
                # Draw bounding box with thicker lines
                color = self._get_class_color(cls)
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 3)
                
                # Draw label with background
                label = f"{class_name}: {score:.2f}"
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(
                    annotated_frame,
                    (x1, y1 - label_size[1] - 10),
                    (x1 + label_size[0] + 5, y1),
                    color,
                    -1
                )
                cv2.putText(
                    annotated_frame,
                    label,
                    (x1 + 2, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )
        
        # Store in history
        self.object_history.append({
            'timestamp': time.time(),
            'detections': detections,
            'count': len(detections)
        })
        
        # Convert back to RGB for display
        annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
        
        # Calculate FPS
        elapsed = time.time() - start_time
        fps = 1.0 / elapsed if elapsed > 0 else 0
        self.fps_history.append(fps)
        
        return annotated_frame, detections
    
    def create_summary_image(self, frame, detections):
        """
        Create a beautiful summary image with detection stats
        Frame is already in RGB format
        """
        # Frame is RGB, convert to BGR for OpenCV operations
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Resize frame to output size
        output = cv2.resize(frame_bgr, (config.OUTPUT_WIDTH, config.OUTPUT_HEIGHT))
        h, w = output.shape[:2]
        
        # Create stats panel at bottom
        panel_height = 150
        panel = np.ones((panel_height, w, 3), dtype=np.uint8) * 40
        
        # Add stats
        y_offset = 30
        avg_fps = np.mean(self.fps_history) if len(self.fps_history) > 0 else 0
        
        stats_lines = [
            f"FPS: {avg_fps:.1f}",
            f"Current Detections: {len(detections)}",
            f"Total Objects Detected: {self.total_detections}",
            f"Unique Classes: {len(self.object_counts)}"
        ]
        
        for i, line in enumerate(stats_lines):
            cv2.putText(
                panel,
                line,
                (20, y_offset + i * 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )
        
        # Add object counts
        x_offset = w // 2
        cv2.putText(
            panel,
            "Object Counts:",
            (x_offset, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2
        )
        
        # Top 5 detected objects
        sorted_objects = sorted(
            self.object_counts.items(),
            key=lambda x: x[1],
            reverse=True
        )[:5]
        
        for i, (obj, count) in enumerate(sorted_objects):
            cv2.putText(
                panel,
                f"{obj}: {count}",
                (x_offset, y_offset + 30 + i * 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )
        
        # Combine
        result = np.vstack([output, panel])
        
        return result
    
    def _get_class_color(self, cls):
        """Get consistent color for each class"""
        np.random.seed(cls)
        color = tuple(map(int, np.random.randint(0, 255, 3)))
        return color
    
    def get_stats(self):
        """Get detection statistics"""
        return {
            'total_detections': self.total_detections,
            'object_counts': dict(self.object_counts),
            'unique_classes': len(self.object_counts),
            'avg_fps': np.mean(self.fps_history) if len(self.fps_history) > 0 else 0
        }
    
    def reset_counts(self):
        """Reset detection counts"""
        self.object_counts.clear()
        self.total_detections = 0
        print("[DETECTOR] Counts reset")
