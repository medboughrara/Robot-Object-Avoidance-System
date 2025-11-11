import cv2
import numpy as np
import serial
import time
import os
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import torch
from pathlib import Path

class RobotController:
    def __init__(self, serial_port='COM3', baud_rate=9600):
        """Initialize the robot controller with serial connection to Arduino"""
        self.serial_conn = None
        self.connected = False
        self.distance = float('inf')
        self.running = False
        self.read_thread = None
        
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to Arduino on {serial_port}")
            self.connected = True
            self.distance = float('inf')
            # Start a thread to read distance data
            self.running = True
            self.read_thread = threading.Thread(target=self._read_distance, daemon=True)
            self.read_thread.start()
        except Exception as e:
            print(f"Warning: Could not connect to Arduino: {e}")
            self.connected = False
            
    def _read_distance(self):
        """Background thread to read distance data from Arduino"""
        while self.running and self.connected:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode().strip()
                    if line.startswith('D:'):  # Distance data
                        try:
                            self.distance = float(line[2:])
                        except ValueError:
                            pass
            except Exception as e:
                print(f"Error reading distance: {e}")
                time.sleep(0.1)
                
    def get_distance(self):
        """Get the last measured distance from the ultrasonic sensor"""
        return self.distance

    def send_command(self, command):
        """Send a command to the Arduino"""
        if not self.connected or not self.serial_conn:
            print(f"Simulating command: {command}")
            return
            
        try:
            self.serial_conn.write(command.encode())
            time.sleep(0.05)
            if self.serial_conn.in_waiting:
                response = self.serial_conn.readline().decode().strip()
                print(f"Arduino: {response}")
        except Exception as e:
            print(f"Error sending command: {e}")

    def execute_timed_command(self, command, duration=2.0):
        """Execute a command for a specific duration then stop"""
        self.send_command(command + "\n")
        time.sleep(duration)
        self.send_command("S\n")
        time.sleep(0.1)

    def move_forward(self):
        """Move forward for 2 seconds"""
        self.execute_timed_command("F")

    def move_backward(self):
        """Move backward for 2 seconds"""
        self.execute_timed_command("B")

    def turn_left(self):
        """Turn left for 2 seconds"""
        self.execute_timed_command("L")

    def turn_right(self):
        """Turn right for 2 seconds"""
        self.execute_timed_command("R")

    def stop(self):
        """Immediate stop"""
        self.send_command("S\n")

    def shutdown(self):
        """Shutdown the robot controller"""
        self.running = False
        if self.connected and self.serial_conn:
            try:
                self.serial_conn.close()
            except:
                pass
        self.connected = False


class YOLOObjectDetector:
    """YOLO-based object detector with SAM segmentation"""
    
    def __init__(self, yolo_cfg_path, yolo_weights_path, yolo_names_path):
        """Initialize YOLO detector"""
        print("Loading YOLO model...")
        
        # Check if files exist
        for file_path in [yolo_cfg_path, yolo_weights_path, yolo_names_path]:
            if not os.path.exists(file_path):
                raise FileNotFoundError(f"Required file not found: {file_path}")
        
        # Load YOLO model
        self.net = cv2.dnn.readNetFromDarknet(yolo_cfg_path, yolo_weights_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        
        # Load classes
        with open(yolo_names_path, 'r') as f:
            self.classes = [line.strip() for line in f]
        
        print(f"YOLO loaded with {len(self.classes)} classes")
        
        # Try to load SAM model
        self.sam_predictor = None
        try:
            print("Loading SAM model...")
            from segment_anything import sam_model_registry, SamPredictor
            
            sam_checkpoint = "sam_vit_b_01ec64.pth"
            if os.path.exists(sam_checkpoint):
                device = "cuda" if torch.cuda.is_available() else "cpu"
                sam = sam_model_registry["vit_b"](checkpoint=sam_checkpoint)
                sam.to(device=device)
                self.sam_predictor = SamPredictor(sam)
                print(f"SAM loaded on {device}")
            else:
                print("SAM checkpoint not found. Using YOLO detections only.")
        except Exception as e:
            print(f"Could not load SAM: {e}. Using YOLO detections only.")
    
    def detect_objects(self, frame):
        """Perform YOLO object detection"""
        height, width = frame.shape[:2]
        
        # Create blob from image
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        
        # Get output layer names
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        
        # Forward pass
        outputs = self.net.forward(output_layers)
        
        # Lists to store detections
        boxes = []
        confidences = []
        class_ids = []
        detected_objects = []
        
        # Process detections
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                if confidence > 0.5:  # Confidence threshold
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    
                    # Ensure bounding box is within frame
                    x = max(0, x)
                    y = max(0, y)
                    
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                    
                    detected_objects.append({
                        'label': self.classes[class_id],
                        'confidence': confidence,
                        'center': (center_x, center_y),
                        'box': (x, y, w, h),
                        'class_id': class_id
                    })
        
        # Apply non-maximum suppression
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        
        # Draw boxes and segmentations
        filtered_detections = []
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = confidences[i]
                
                # Draw rectangle
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f'{label} {confidence:.2f}', 
                           (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (0, 255, 0), 2)
                
                # Use SAM for segmentation if available
                if self.sam_predictor:
                    try:
                        self.sam_predictor.set_image(frame)
                        # Use bounding box as prompt for SAM
                        input_box = np.array([x, y, x + w, y + h])
                        masks, scores, logits = self.sam_predictor.predict(
                            box=input_box,
                            multimask_output=False
                        )
                        
                        if masks is not None and len(masks) > 0:
                            mask = masks[0].astype(np.uint8) * 255
                            # Apply colored overlay
                            colored_mask = np.zeros_like(frame)
                            colored_mask[:, :, 1] = mask  # Green channel
                            frame = cv2.addWeighted(frame, 0.8, colored_mask, 0.2, 0)
                    except Exception as e:
                        print(f"SAM segmentation error: {e}")
                
                filtered_detections.append(detected_objects[i])
        
        return frame, filtered_detections


class RobotControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("YOLO + SAM Robot Controller")
        self.root.geometry("1400x850")
        
        self.robot = None
        self.camera_index = 0
        self.cap = None
        self.detector = None
        self.running = False
        self.auto_mode = False
        self.last_movement_time = time.time()
        self.movement_interval = 2.5  # seconds between movements
        self.current_action = "Idle"
        
        self.setup_ui()
        self.initialize_detector()
        self.list_cameras()
        
    def initialize_detector(self):
        """Initialize YOLO detector"""
        try:
            yolo_path = os.path.join(os.path.dirname(__file__), 'yolo_files')
            cfg_path = os.path.join(yolo_path, 'yolov3.cfg')
            weights_path = os.path.join(yolo_path, 'yolov3.weights')
            names_path = os.path.join(yolo_path, 'coco.names')
            
            self.detector = YOLOObjectDetector(cfg_path, weights_path, names_path)
            self.log_status("YOLO model loaded successfully")
        except Exception as e:
            self.log_status(f"Error loading YOLO: {e}")
            messagebox.showerror("Error", f"Failed to load YOLO model:\n{e}")
        
    def setup_ui(self):
        """Setup the GUI interface"""
        # Control panel on the left
        control_frame = ttk.Frame(self.root, width=350)
        control_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=10, pady=10)
        
        # Title
        title_label = ttk.Label(control_frame, text="YOLO + SAM Robot", font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # Camera selection
        ttk.Label(control_frame, text="Select Camera:", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        self.camera_listbox = tk.Listbox(control_frame, height=4)
        self.camera_listbox.pack(fill=tk.BOTH, expand=False, pady=5)
        
        # Buttons to open/close camera
        camera_button_frame = ttk.Frame(control_frame)
        camera_button_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(camera_button_frame, text="Connect", command=self.connect_camera).pack(side=tk.LEFT, padx=2)
        ttk.Button(camera_button_frame, text="Stop", command=self.stop_camera).pack(side=tk.LEFT, padx=2)
        
        # Arduino connection
        ttk.Label(control_frame, text="Arduino Port:", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        self.arduino_port_var = tk.StringVar(value="COM3")
        self.port_combobox = ttk.Combobox(control_frame, textvariable=self.arduino_port_var,
                                         values=["COM1", "COM2", "COM3", "COM4", "/dev/ttyACM0", "/dev/ttyUSB0"],
                                         state="normal")
        self.port_combobox.pack(fill=tk.X, pady=5)
        
        # Connect/Disconnect Arduino buttons
        arduino_button_frame = ttk.Frame(control_frame)
        arduino_button_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(arduino_button_frame, text="Connect", command=self.connect_arduino).pack(side=tk.LEFT, padx=2)
        ttk.Button(arduino_button_frame, text="Disconnect", command=self.disconnect_arduino).pack(side=tk.LEFT, padx=2)
        
        # Status display
        ttk.Label(control_frame, text="Status Log:", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        self.status_text = tk.Text(control_frame, height=8, width=40, state=tk.DISABLED, font=("Courier", 8))
        self.status_text.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Manual Control Section
        ttk.Label(control_frame, text="Manual Control:", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.BOTH, expand=False, pady=5)
        
        ttk.Button(button_frame, text="↑ Forward", command=lambda: self.manual_control("F")).pack(fill=tk.X, pady=2)
        
        lr_frame = ttk.Frame(button_frame)
        lr_frame.pack(fill=tk.X, pady=2)
        ttk.Button(lr_frame, text="← Left", command=lambda: self.manual_control("L")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
        ttk.Button(lr_frame, text="STOP", command=lambda: self.manual_control("S")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
        ttk.Button(lr_frame, text="Right →", command=lambda: self.manual_control("R")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
        
        ttk.Button(button_frame, text="↓ Backward", command=lambda: self.manual_control("B")).pack(fill=tk.X, pady=2)
        
        # Detection stats
        ttk.Label(control_frame, text="Detection Stats:", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        stats_frame = ttk.Frame(control_frame)
        stats_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(stats_frame, text="Objects:").pack(side=tk.LEFT)
        self.objects_label = ttk.Label(stats_frame, text="0", font=("Arial", 10, "bold"))
        self.objects_label.pack(side=tk.LEFT, padx=5)
        
        # Distance display
        ttk.Label(control_frame, text="Distance (cm):", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        self.distance_label = ttk.Label(control_frame, text="N/A", font=("Arial", 12, "bold"))
        self.distance_label.pack(pady=5)
        
        # Current action
        ttk.Label(control_frame, text="Current Action:", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        self.action_label = ttk.Label(control_frame, text="Idle", font=("Arial", 10), foreground="blue")
        self.action_label.pack(pady=5)
        
        # Auto mode
        ttk.Label(control_frame, text="Auto Mode:", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        self.auto_mode_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(control_frame, text="Enable Auto Control", 
                       variable=self.auto_mode_var, command=self.toggle_auto_mode).pack(fill=tk.X, pady=5)
        
        # Video display on the right
        video_frame = ttk.LabelFrame(self.root, text="YOLO Detection + SAM Segmentation")
        video_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.video_label = ttk.Label(video_frame, text="Camera Feed", background="black")
        self.video_label.pack(fill=tk.BOTH, expand=True)
        
        # Start the main loop
        self.update_status()
        self.update_video()
        
    def list_cameras(self):
        """List available camera devices"""
        print("Searching for available cameras...")
        available_cameras = []
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    print(f"Camera {i} is available")
                    available_cameras.append(f"Camera {i}")
                cap.release()
        
        self.camera_listbox.delete(0, tk.END)
        for camera in available_cameras:
            self.camera_listbox.insert(tk.END, camera)
        
        if available_cameras:
            self.camera_listbox.selection_set(0)
            
    def get_selected_camera(self):
        """Get the selected camera index"""
        selection = self.camera_listbox.curselection()
        if selection:
            return int(self.camera_listbox.get(selection[0]).split()[-1])
        return 0
        
    def connect_camera(self):
        """Connect to selected camera"""
        self.camera_index = self.get_selected_camera()
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            messagebox.showerror("Error", f"Could not open camera {self.camera_index}")
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        self.running = True
        self.log_status(f"Camera {self.camera_index} connected")
        
    def stop_camera(self):
        """Stop the camera"""
        self.running = False
        if self.cap:
            self.cap.release()
            self.cap = None
        self.log_status("Camera stopped")
        
    def connect_arduino(self):
        """Connect to Arduino"""
        port = self.arduino_port_var.get()
        self.robot = RobotController(serial_port=port)
        
        if self.robot.connected:
            self.log_status(f"Arduino connected: {port}")
        else:
            self.log_status(f"Arduino simulation mode (no device on {port})")
            
    def disconnect_arduino(self):
        """Disconnect from Arduino"""
        if self.robot:
            self.robot.shutdown()
            self.robot = None
        self.log_status("Arduino disconnected")
        
    def manual_control(self, command):
        """Send manual control command"""
        if not self.robot:
            messagebox.showwarning("Warning", "Arduino not connected")
            return
        
        command_map = {
            'F': ('Forward', self.robot.move_forward),
            'B': ('Backward', self.robot.move_backward),
            'L': ('Left', self.robot.turn_left),
            'R': ('Right', self.robot.turn_right),
            'S': ('Stop', self.robot.stop)
        }
        
        name, func = command_map.get(command, ('Unknown', None))
        if func:
            self.current_action = f"Manual: {name}"
            self.log_status(f"Manual: {name}")
            func()
            
    def toggle_auto_mode(self):
        """Toggle automatic mode"""
        self.auto_mode = self.auto_mode_var.get()
        if self.auto_mode:
            self.log_status("Auto mode ENABLED")
        else:
            self.log_status("Auto mode DISABLED")
            if self.robot:
                self.robot.stop()
            self.current_action = "Idle"
                
    def log_status(self, message):
        """Log status message"""
        self.status_text.config(state=tk.NORMAL)
        timestamp = time.strftime("%H:%M:%S")
        self.status_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.status_text.see(tk.END)
        self.status_text.config(state=tk.DISABLED)
        
    def update_status(self):
        """Update status information"""
        if self.robot:
            distance = self.robot.get_distance()
            if distance != float('inf'):
                self.distance_label.config(text=f"{distance:.1f}")
            else:
                self.distance_label.config(text="N/A")
        
        self.action_label.config(text=self.current_action)
        self.root.after(500, self.update_status)
        
    def update_video(self):
        """Update video feed"""
        if self.running and self.cap and self.detector:
            ret, frame = self.cap.read()
            if ret:
                # Detect objects with YOLO and SAM
                frame, detections = self.detector.detect_objects(frame)
                
                # Update detection count
                self.objects_label.config(text=str(len(detections)))
                
                # Auto control logic
                if self.auto_mode and self.robot:
                    current_time = time.time()
                    if current_time - self.last_movement_time >= self.movement_interval:
                        self.execute_auto_control(detections, frame)
                        self.last_movement_time = current_time
                
                # Add info
                cv2.putText(frame, f"Detected: {len(detections)} objects", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if self.auto_mode:
                    cv2.putText(frame, "AUTO MODE ENABLED", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Convert to PhotoImage
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                img.thumbnail((800, 600), Image.Resampling.LANCZOS)
                photo = ImageTk.PhotoImage(img)
                
                self.video_label.config(image=photo)
                self.video_label.image = photo
        
        self.root.after(30, self.update_video)
        
    def execute_auto_control(self, detections, frame):
        """Execute automatic robot control based on YOLO+SAM detections"""
        if not self.robot:
            return
        
        distance = self.robot.get_distance()
        
        # Emergency stop based on distance sensor
        if distance < 20:
            self.current_action = "EMERGENCY STOP"
            self.log_status(f"🚨 EMERGENCY STOP! Distance: {distance:.1f}cm")
            self.robot.move_backward()
            return
        
        frame_width = frame.shape[1]
        frame_center = frame_width / 2
        
        if detections:
            # Get the largest detected object (closest/most important)
            largest = max(detections, key=lambda x: x['box'][2] * x['box'][3])
            obj_center_x = largest['center'][0]
            obj_width = largest['box'][2]
            obj_height = largest['box'][3]
            obj_label = largest['label']
            
            # Calculate offset from center
            offset = obj_center_x - frame_center
            
            # Check distance sensor
            if distance < 40 and distance != float('inf'):
                self.current_action = f"Avoiding (distance: {distance:.1f}cm)"
                self.log_status(f"⚠️  WARNING DISTANCE: {distance:.1f}cm - Turning")
                if offset < 0:
                    self.robot.turn_right()
                else:
                    self.robot.turn_left()
            
            # Check if object is too close based on bounding box size
            elif obj_height > frame.shape[0] * 0.4 or obj_width > frame.shape[1] * 0.4:
                self.current_action = f"Object too close: {obj_label}"
                self.log_status(f"📦 LARGE OBJECT DETECTED ({obj_label}) - Backing up")
                self.robot.move_backward()
            
            # Check if object is off-center
            elif abs(offset) > 120:
                if offset < 0:
                    self.current_action = f"Turning right (avoiding {obj_label})"
                    self.log_status(f"⬅️  Object on LEFT ({obj_label}) - Turning RIGHT")
                    self.robot.turn_right()
                else:
                    self.current_action = f"Turning left (avoiding {obj_label})"
                    self.log_status(f"➡️  Object on RIGHT ({obj_label}) - Turning LEFT")
                    self.robot.turn_left()
            
            # Path looks clear
            else:
                self.current_action = f"Moving forward (detected: {obj_label})"
                self.log_status(f"✓ CLEAR PATH - Moving forward (near: {obj_label})")
                self.robot.move_forward()
        
        else:
            # No objects detected
            if distance > 100 or distance == float('inf'):
                self.current_action = "Forward (clear path)"
                self.log_status("✓ COMPLETELY CLEAR - Moving forward")
                self.robot.move_forward()
            else:
                self.current_action = f"Cautious forward (distance: {distance:.1f}cm)"
                self.log_status(f"⚠️  Cautious forward (distance: {distance:.1f}cm)")
                self.robot.move_forward()


def main():
    root = tk.Tk()
    gui = RobotControllerGUI(root)
    
    def on_closing():
        if gui.running:
            gui.stop_camera()
        if gui.robot:
            gui.disconnect_arduino()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
