import cv2
import numpy as np
from ultralytics import YOLO
from pathlib import Path
import os

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_ROOT / "models" / "yolov8n.pt"

class Detector:
    def __init__(self):
        print(f"Loading YOLO model from {MODEL_PATH}")
        self.model = YOLO(str(MODEL_PATH))
        self.target_class = "person"  # default target
        self.confidence_threshold = 0.10
        
        # Color thresholds for geometric shapes (HSV)
        self.color_ranges = {
            "red_sphere": (np.array([0, 100, 100]), np.array([10, 255, 255])),
            "blue_cube": (np.array([100, 100, 100]), np.array([140, 255, 255])),
            "green_cone": (np.array([35, 40, 40]), np.array([85, 255, 255])),
            "yellow_cylinder": (np.array([20, 100, 100]), np.array([35, 255, 255])),
            "person": (np.array([5, 100, 100]), np.array([20, 255, 255])), # Orange color bounds
            "car": (np.array([140, 100, 100]), np.array([170, 255, 255])), # Magenta color bounds
        }

    def set_target(self, target_class):
        self.target_class = target_class

    def process_frame(self, frame):
        if frame is None:
            return frame, None
            
        annotated_frame = frame.copy()
        detection_info = None
        
        target = self.target_class
        
        # 1. Use YOLO for COCO classes (like 'person')
        if target not in self.color_ranges:
            results = self.model(frame, verbose=False, conf=self.confidence_threshold)
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = self.model.names[cls_id]
                    
                    if class_name == target and conf >= self.confidence_threshold:
                        # Get bounding box
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        w = x2 - x1
                        h = y2 - y1
                        
                        # Calculate distance (rough estimation based on pixel height)
                        # Assumes focal length ~ 277 (from camera config) and object height ~1.7m (person)
                        # Dist = (Real_Height * Focal_Length) / Pixel_Height
                        # Using a simplified constant for demonstration
                        dist = 400.0 / h if h > 0 else -1
                        
                        detection_info = {
                            "class": class_name,
                            "confidence": conf,
                            "bbox": (x1, y1, w, h),
                            "center": (x1 + w//2, y1 + h//2),
                            "distance": dist
                        }
                        
                        # Draw bounding box
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"{class_name} {conf:.2f} (d: {dist:.1f}m)"
                        cv2.putText(annotated_frame, label, (x1, max(y1-10, 0)), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        # Only track the most confident/first one
                        break 
                        
        # 2. Use OpenCV color masking for custom geometric shapes
        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower, upper = self.color_ranges[target]
            mask = cv2.inRange(hsv, lower, upper)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 100:  # Minimum size threshold
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    
                    # Distance estimation (rough)
                    if target == "car":
                        dist = 500.0 / max(w, h) if max(w, h) > 0 else -1
                    else:
                        dist = 200.0 / max(w, h) if max(w, h) > 0 else -1
                    
                    detection_info = {
                        "class": target,
                        "confidence": 1.0,
                        "bbox": (x, y, w, h),
                        "center": (x + w//2, y + h//2),
                        "distance": dist
                    }
                    
                    # Draw
                    cv2.rectangle(annotated_frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    label = f"{target} (d: {dist:.1f}m)"
                    cv2.putText(annotated_frame, label, (x, max(y-10, 0)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Draw crosshair in center
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.line(annotated_frame, (cx - 10, cy), (cx + 10, cy), (255, 255, 255), 1)
        cv2.line(annotated_frame, (cx, cy - 10), (cx, cy + 10), (255, 255, 255), 1)
        
        return annotated_frame, detection_info
