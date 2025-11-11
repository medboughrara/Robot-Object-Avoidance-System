import cv2
import numpy as np
import serial
import time
import os
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import io

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
            print(f"Warning: Arduino not connected, simulating command: {command}")
            return
            
        try:
            self.serial_conn.write(command.encode())
            time.sleep(0.1)
            if self.serial_conn.in_waiting:
                response = self.serial_conn.readline().decode().strip()
                print(f"Arduino response: {response}")
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


class SimpleObjectDetector:
    """Simple motion-based object detector using color and contours (no YOLO required)"""
    
    def __init__(self):
        self.lower_hsv = np.array([0, 50, 50])
        self.upper_hsv = np.array([180, 255, 255])
        
    def detect_objects(self, frame):
        """Detect moving objects using color-based segmentation"""
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for colored objects
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        
        # Apply morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Draw rectangle
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                
                detections.append({
                    'label': 'Object',
                    'center': (center_x, center_y),
                    'box': (x, y, w, h),
                    'area': area
                })
        
        return frame, detections


class RobotControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Webcam Controller")
        self.root.geometry("1200x800")
        
        self.robot = None
        self.camera_index = 0
        self.cap = None
        self.detector = SimpleObjectDetector()
        self.running = False
        self.auto_mode = False
        self.last_movement_time = time.time()
        self.movement_interval = 2.5  # seconds between movements
        
        self.setup_ui()
        self.list_cameras()
        
    def setup_ui(self):
        """Setup the GUI interface"""
        # Control panel on the left
        control_frame = ttk.Frame(self.root, width=300)
        control_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=10, pady=10)
        
        # Title
        title_label = ttk.Label(control_frame, text="Robot Controller", font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # Camera selection
        ttk.Label(control_frame, text="Select Camera:", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        self.camera_listbox = tk.Listbox(control_frame, height=5)
        self.camera_listbox.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Buttons to open/close camera
        camera_button_frame = ttk.Frame(control_frame)
        camera_button_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(camera_button_frame, text="Connect Camera", command=self.connect_camera).pack(side=tk.LEFT, padx=2)
        ttk.Button(camera_button_frame, text="Stop Camera", command=self.stop_camera).pack(side=tk.LEFT, padx=2)
        
        # Arduino connection
        ttk.Label(control_frame, text="Arduino Port:", font=("Arial", 10, "bold")).pack(pady=(15, 5))
        self.arduino_port_var = tk.StringVar(value="COM3")
        self.port_combobox = ttk.Combobox(control_frame, textvariable=self.arduino_port_var,
                                         values=["COM1", "COM2", "COM3", "COM4", "/dev/ttyACM0", "/dev/ttyUSB0"],
                                         state="normal")
        self.port_combobox.pack(fill=tk.X, pady=5)
        
        # Connect/Disconnect Arduino buttons
        arduino_button_frame = ttk.Frame(control_frame)
        arduino_button_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(arduino_button_frame, text="Connect Arduino", command=self.connect_arduino).pack(side=tk.LEFT, padx=2)
        ttk.Button(arduino_button_frame, text="Disconnect", command=self.disconnect_arduino).pack(side=tk.LEFT, padx=2)
        
        # Status display
        ttk.Label(control_frame, text="Status:", font=("Arial", 10, "bold")).pack(pady=(15, 5))
        self.status_text = tk.Text(control_frame, height=6, width=35, state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Manual Control Section
        ttk.Label(control_frame, text="Manual Control:", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        
        # Direction buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Up button
        ttk.Button(button_frame, text="↑ Forward", command=lambda: self.manual_control("F")).pack(fill=tk.X, pady=2)
        
        # Left, Stop, Right
        lr_frame = ttk.Frame(button_frame)
        lr_frame.pack(fill=tk.X, pady=2)
        ttk.Button(lr_frame, text="← Left", command=lambda: self.manual_control("L")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
        ttk.Button(lr_frame, text="STOP", command=lambda: self.manual_control("S")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
        ttk.Button(lr_frame, text="Right →", command=lambda: self.manual_control("R")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
        
        # Down button
        ttk.Button(button_frame, text="↓ Backward", command=lambda: self.manual_control("B")).pack(fill=tk.X, pady=2)
        
        # Auto mode
        ttk.Label(control_frame, text="Auto Mode:", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        self.auto_mode_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(control_frame, text="Enable Auto Detection Mode", 
                       variable=self.auto_mode_var, command=self.toggle_auto_mode).pack(fill=tk.X, pady=5)
        
        # Distance display (if available)
        ttk.Label(control_frame, text="Distance (cm):", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        self.distance_label = ttk.Label(control_frame, text="N/A", font=("Arial", 12, "bold"))
        self.distance_label.pack(pady=5)
        
        # Video display on the right
        video_frame = ttk.Frame(self.root)
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
            self.log_status(f"Arduino connected on {port}")
        else:
            self.log_status(f"Arduino connection failed (simulating mode)")
            
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
            self.log_status(f"Manual: {name}")
            func()
            
    def toggle_auto_mode(self):
        """Toggle automatic mode"""
        self.auto_mode = self.auto_mode_var.get()
        if self.auto_mode:
            self.log_status("Auto mode enabled")
        else:
            self.log_status("Auto mode disabled")
            if self.robot:
                self.robot.stop()
                
    def log_status(self, message):
        """Log status message"""
        self.status_text.config(state=tk.NORMAL)
        self.status_text.insert(tk.END, f"{message}\n")
        self.status_text.see(tk.END)
        self.status_text.config(state=tk.DISABLED)
        
    def update_status(self):
        """Update status information"""
        if self.robot:
            distance = self.robot.get_distance()
            if distance != float('inf'):
                self.distance_label.config(text=f"{distance:.1f} cm")
            else:
                self.distance_label.config(text="N/A")
        
        self.root.after(500, self.update_status)
        
    def update_video(self):
        """Update video feed"""
        if self.running and self.cap:
            ret, frame = self.cap.read()
            if ret:
                # Detect objects
                frame, detections = self.detector.detect_objects(frame)
                
                # Auto control logic
                if self.auto_mode and self.robot:
                    current_time = time.time()
                    if current_time - self.last_movement_time >= self.movement_interval:
                        self.execute_auto_control(detections, frame)
                        self.last_movement_time = current_time
                
                # Add detection info
                if detections:
                    cv2.putText(frame, f"Detected: {len(detections)} object(s)", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Convert to PhotoImage
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                img.thumbnail((700, 550), Image.Resampling.LANCZOS)
                photo = ImageTk.PhotoImage(img)
                
                self.video_label.config(image=photo)
                self.video_label.image = photo
        
        self.root.after(30, self.update_video)
        
    def execute_auto_control(self, detections, frame):
        """Execute automatic robot control based on detections"""
        if not self.robot:
            return
        
        distance = self.robot.get_distance()
        
        # Emergency stop
        if distance < 20:
            self.log_status(f"Emergency stop! Distance: {distance:.1f}cm")
            self.robot.move_backward()
            return
        
        frame_width = frame.shape[1]
        frame_center = frame_width / 2
        
        if detections:
            # Get the largest detected object
            largest = max(detections, key=lambda x: x['area'])
            obj_center_x = largest['center'][0]
            obj_width = largest['box'][2]
            
            # Calculate offset from center
            offset = obj_center_x - frame_center
            
            if distance < 40:  # Warning distance
                self.log_status(f"Warning! Close distance: {distance:.1f}cm")
                if offset < 0:
                    self.robot.turn_right()
                else:
                    self.robot.turn_left()
            elif obj_width > frame.shape[1] * 0.3:  # Object too close based on size
                self.log_status("Object too large - Moving backward")
                self.robot.move_backward()
            elif abs(offset) > 100:  # Object off-center
                if offset < 0:
                    self.log_status("Object on left - Turning right")
                    self.robot.turn_right()
                else:
                    self.log_status("Object on right - Turning left")
                    self.robot.turn_left()
            else:  # Clear path
                self.log_status("Path clear - Moving forward")
                self.robot.move_forward()
        else:
            # No objects detected
            if distance > 100:
                self.log_status("Clear path - Moving forward")
                self.robot.move_forward()
            else:
                self.log_status("Caution - Moving forward slowly")
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
