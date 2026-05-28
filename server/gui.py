"""
Improved GUI for robot server - No Flicker Version
"""
import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import threading
import time

class RobotGUI:
    def __init__(self, title="Robot Control Server"):
        self.root = tk.Tk()
        self.root.title(title)
        self.root.geometry("1400x900")
        self.root.configure(bg='#1e1e1e')
        
        # Internal state for smooth rendering
        self.canvas_image_id = None  # Persistent ID for the image object
        self.detection_image = None  # Must keep reference to avoid garbage collection
        
        # Style
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('Title.TLabel', background='#1e1e1e', foreground='#00ff00', font=('Arial', 16, 'bold'))
        self.style.configure('Info.TLabel', background='#2d2d2d', foreground='#ffffff', font=('Arial', 10))
        self.style.configure('Status.TLabel', background='#2d2d2d', foreground='#00ff00', font=('Arial', 9))
        
        # Main container
        main_frame = tk.Frame(self.root, bg='#1e1e1e')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Top status bar
        status_frame = tk.Frame(main_frame, bg='#2d2d2d', height=60)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        status_frame.pack_propagate(False)
        
        self.status_label = ttk.Label(status_frame, text="⚫ Waiting for connections...", style='Title.TLabel')
        self.status_label.pack(side=tk.LEFT, padx=20, pady=15)
        
        self.mode_label = ttk.Label(status_frame, text="Mode: Unknown", style='Info.TLabel')
        self.mode_label.pack(side=tk.RIGHT, padx=20, pady=15)
        
        # Content area
        content_frame = tk.Frame(main_frame, bg='#1e1e1e')
        content_frame.pack(fill=tk.BOTH, expand=True)
        
        # Left side - Object Detection
        left_frame = tk.Frame(content_frame, bg='#2d2d2d')
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        detection_title = ttk.Label(left_frame, text="🎯 Object Detection", style='Title.TLabel')
        detection_title.pack(pady=10)
        
        self.detection_canvas = tk.Canvas(left_frame, bg='#000000', highlightthickness=0)
        self.detection_canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Detection stats
        stats_frame = tk.Frame(left_frame, bg='#2d2d2d', height=100)
        stats_frame.pack(fill=tk.X, padx=10, pady=10)
        stats_frame.pack_propagate(False)
        
        self.detection_stats = ttk.Label(stats_frame, text="Waiting for camera...", style='Info.TLabel', justify=tk.LEFT)
        self.detection_stats.pack(pady=10, padx=10)
        
        # Connection status indicators
        self.connection_frame = tk.Frame(status_frame, bg='#2d2d2d')
        self.connection_frame.pack(side=tk.LEFT, padx=50)
        
        self.esp32_status = ttk.Label(self.connection_frame, text="⚫ ESP32", style='Status.TLabel')
        self.esp32_status.grid(row=0, column=0, padx=10)
        
        self.camera_status = ttk.Label(self.connection_frame, text="⚫ Camera", style='Status.TLabel')
        self.camera_status.grid(row=0, column=1, padx=10)
        
        self.app_status = ttk.Label(self.connection_frame, text="⚫ App", style='Status.TLabel')
        self.app_status.grid(row=0, column=2, padx=10)
        
        self.running = True

    def update_detection_display(self, frame):
        """Update object detection display without flickering"""
        if frame is None:
            return
            
        try:
            # Resize logic
            canvas_width = self.detection_canvas.winfo_width()
            canvas_height = self.detection_canvas.winfo_height()
            
            if canvas_width > 1 and canvas_height > 1:
                h, w = frame.shape[:2]
                aspect = w / h
                
                if canvas_width / canvas_height > aspect:
                    new_height = canvas_height - 20
                    new_width = int(new_height * aspect)
                else:
                    new_width = canvas_width - 20
                    new_height = int(new_width / aspect)
                
                frame_resized = cv2.resize(frame, (new_width, new_height))
                
                # Convert to PhotoImage
                img = Image.fromarray(frame_resized)
                photo = ImageTk.PhotoImage(image=img)
                
                # SMOOTH UPDATE LOGIC:
                x = (canvas_width - new_width) // 2
                y = (canvas_height - new_height) // 2
                
                if self.canvas_image_id is None:
                    # First time: Create the image object
                    self.canvas_image_id = self.detection_canvas.create_image(x, y, anchor=tk.NW, image=photo)
                else:
                    # Subsequent times: Update the EXISTING object (Prevents Flicker)
                    self.detection_canvas.itemconfig(self.canvas_image_id, image=photo)
                    self.detection_canvas.coords(self.canvas_image_id, x, y)
                
                # CRITICAL: Keep reference so garbage collector doesn't delete the image
                self.detection_image = photo
                
        except Exception as e:
            print(f"[GUI] Detection display error: {e}")

    def update_status(self, message, mode=None):
        self.status_label.config(text=message)
        if mode:
            self.mode_label.config(text=f"Mode: {mode}")

    def update_detection_stats(self, stats_text):
        self.detection_stats.config(text=stats_text)

    def update_connections(self, esp32=False, camera=False, app=False):
        self.esp32_status.config(text="🟢 ESP32" if esp32 else "⚫ ESP32")
        self.camera_status.config(text="🟢 Camera" if camera else "⚫ Camera")
        self.app_status.config(text="🟢 App" if app else "⚫ App")

    def run(self):
        self.root.mainloop()
        self.running = False

    def update(self):
        if self.running:
            self.root.update()
