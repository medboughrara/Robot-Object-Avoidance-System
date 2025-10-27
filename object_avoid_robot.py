import cv2
import numpy as np
import serial
import time
import os
import argparse
import threading

class RobotController:
    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=9600):
        """Initialize the robot controller with serial connection to Arduino"""
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to Arduino on {serial_port}")
            self.distance = float('inf')  # Initialize distance to infinity
            # Start a thread to read distance data
            self.running = True
            self.read_thread = threading.Thread(target=self._read_distance)
            self.read_thread.daemon = True
            self.read_thread.start()
        except Exception as e:
            print(f"Error connecting to Arduino: {e}")
            print("If you're on Windows, try using 'COM3' (or another COM port)")
            print("If you're on Linux, try using '/dev/ttyACM0' or '/dev/ttyUSB0'")
            exit(1)
            
    def _read_distance(self):
        """Background thread to read distance data from Arduino"""
        while self.running:
            try:
                if self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode().strip()
                    if line.startswith('D:'):  # Distance data
                        try:
                            self.distance = float(line[2:])
                        except ValueError:
                            pass  # Ignore invalid distance values
            except Exception as e:
                print(f"Error reading distance: {e}")
                time.sleep(0.1)
                
    def get_distance(self):
        """Get the last measured distance from the ultrasonic sensor"""
        return self.distance

    def send_command(self, command):
        """Send a command to the Arduino"""
        try:
            self.serial_conn.write(command.encode())
            time.sleep(0.1)  # Give Arduino time to process
            response = self.serial_conn.readline().decode().strip()
            print(f"Arduino response: {response}")
        except Exception as e:
            print(f"Error sending command: {e}")

    def execute_timed_command(self, command, duration=2.0):
        """Execute a command for a specific duration then stop"""
        self.send_command(command + "\n")
        time.sleep(duration)  # Wait for specified duration
        self.send_command("S\n")  # Stop after duration
        time.sleep(0.1)  # Small delay after stop

    def move_forward(self):
        self.execute_timed_command("F")  # Forward for 2 seconds

    def move_backward(self):
        self.execute_timed_command("B")  # Backward for 2 seconds

    def turn_left(self):
        self.execute_timed_command("L")  # Left for 2 seconds

    def turn_right(self):
        self.execute_timed_command("R")  # Right for 2 seconds

    def stop(self):
        self.send_command("S\n")  # Immediate stop

class ObjectDetector:
    def __init__(self):
        """Initialize the YOLO object detector"""
        # Define the path to YOLO files
        yolo_path = os.path.join(os.path.dirname(__file__), 'yolo_files')
        cfg_path = os.path.join(yolo_path, 'yolov3.cfg')
        weights_path = os.path.join(yolo_path, 'yolov3.weights')
        names_path = os.path.join(yolo_path, 'coco.names')

        # Check if files exist
        for file_path in [cfg_path, weights_path, names_path]:
            if not os.path.exists(file_path):
                print(f"Error: Required file not found: {file_path}")
                exit(1)

        # Load YOLO model
        self.net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)
        
        # Load classes
        with open(names_path, 'r') as f:
            self.classes = [line.strip() for line in f]

    def detect_objects(self, frame):
        """Perform object detection on a frame"""
        height, width = frame.shape[:2]
        
        # Create blob and set as input
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
                    
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                    
                    # Store object info
                    detected_objects.append({
                        'label': self.classes[class_id],
                        'confidence': confidence,
                        'center': (center_x, center_y),
                        'box': (x, y, w, h)
                    })
        
        # Apply non-maximum suppression
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        
        # Draw boxes and return filtered detections
        filtered_detections = []
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = confidences[i]
                
                # Draw rectangle and label
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f'{label} {confidence:.2f}', 
                           (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (0, 255, 0), 2)
                
                filtered_detections.append(detected_objects[i])
        
        return frame, filtered_detections

def list_available_cameras():
    """List all available camera devices"""
    available_cameras = []
    for i in range(10):  # Check first 10 camera indices
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"Camera {i} is available")
                available_cameras.append(i)
            cap.release()
    return available_cameras

def main():
    # List available cameras
    print("\nSearching for available cameras...")
    available_cameras = list_available_cameras()
    
    if not available_cameras:
        print("No cameras found!")
        exit(1)
    
    print("\nAvailable cameras:", available_cameras)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Robot Object Avoidance')
    parser.add_argument('--camera', type=int, default=available_cameras[0],
                        help=f'Camera index (default: {available_cameras[0]})')
    parser.add_argument('--serial-port', type=str,
                        default='/dev/ttyACM0' if os.name != 'nt' else 'COM3',
                        help='Serial port for Arduino')
    args = parser.parse_args()

    # Initialize robot controller and object detector
    robot = RobotController(serial_port=args.serial_port)
    detector = ObjectDetector()

    # Initialize camera with default resolution first
    print(f"\nTrying to open camera {args.camera}")
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)  # Try DirectShow backend
    
    if not cap.isOpened():
        print(f"Failed with DirectShow, trying default backend...")
        cap = cv2.VideoCapture(args.camera)
        
    if not cap.isOpened():
        print(f"Error: Could not open camera {args.camera}")
        exit(1)

    # Wait a bit for camera to initialize
    time.sleep(2)
    
    # Try to read a test frame
    ret, test_frame = cap.read()
    if not ret or test_frame is None:
        print("Error: Camera opened but cannot read frames")
        print("Try disconnecting and reconnecting the camera")
        cap.release()
        exit(1)
    
    print("Camera successfully initialized!")
    
    # Now set the properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # Start with higher resolution
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Standard resolution
    cap.set(cv2.CAP_PROP_FPS, 120)            # Standard FPS
    
    # Verify what settings were actually applied
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    print(f"Camera configured with resolution: {width}x{height} at {fps} FPS")
    
    # If we got a valid frame, show its size
    print(f"First frame shape: {test_frame.shape}")

    print("Starting object avoidance system...")
    print("Press 'q' to quit")

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("Error: Can't receive frame from camera")
                time.sleep(0.1)  # Wait a bit before trying again
                continue
                
            # Check if frame is empty or all black
            if frame.size == 0 or np.mean(frame) < 1.0:
                print("Warning: Received empty or black frame")
                time.sleep(0.1)  # Wait a bit before trying again
                continue

            # Detect objects
            frame, detections = detector.detect_objects(frame)
            
            # Get current distance from ultrasonic sensor
            distance = robot.get_distance()
            
            # Process detections and control robot
            if distance < 20:  # Emergency stop distance (20cm)
                print(f"Emergency stop! Distance: {distance}cm")
                robot.move_backward()  # Emergency backward movement
                continue  # Skip to next frame
                
            # Process vision-based movement every 2.5 seconds
            current_time = time.time()
            if not hasattr(main, 'last_movement_time'):
                main.last_movement_time = current_time - 2.5  # Initialize
                
            if current_time - main.last_movement_time >= 2.5:  # 2.5s cycle (2s movement + 0.5s pause)
                if distance < 40:  # Warning distance (40cm)
                    print(f"Warning! Object too close: {distance}cm")
                    # Check camera detection for direction to turn
                    if detections:
                        detection = detections[0]
                        object_center_x = detection['center'][0]
                        frame_center_x = frame.shape[1] / 2
                        if object_center_x < frame_center_x:
                            print("Obstacle on left - Moving right for 2 seconds")
                            robot.turn_right()
                        else:
                            print("Obstacle on right - Moving left for 2 seconds")
                            robot.turn_left()
                    else:
                        print("Warning distance - Moving backward for 2 seconds")
                        robot.move_backward()
                else:
                    # Safe distance, use camera for navigation
                    if detections:
                        # Get the closest/largest object
                        frame_center_x = frame.shape[1] / 2
                        detection = detections[0]
                        object_center_x = detection['center'][0]
                        
                        # Calculate distance from center
                        distance_x = object_center_x - frame_center_x
                        
                        # Implement avoidance logic
                        if abs(distance_x) > 100:  # Object is significantly off-center
                            if distance_x < 0:
                                print("Object on left - Moving right for 2 seconds")
                                robot.turn_right()
                            else:
                                print("Object on left - Moving left for 2 seconds")
                                robot.turn_left()
                        else:
                            # If object is too close (based on box height)
                            if detection['box'][3] > frame.shape[0] * 0.4:
                                print("Object too close - Moving backward for 2 seconds")
                                robot.move_backward()
                            else:
                                print(f"Path clear (Distance: {distance}cm) - Moving forward for 2 seconds")
                                robot.move_forward()
                    else:
                        # No objects detected, check distance
                        if distance > 100:  # Very clear path
                            print(f"Clear path (Distance: {distance}cm) - Moving forward for 2 seconds")
                            robot.move_forward()
                        else:
                            print(f"Caution (Distance: {distance}cm) - Moving forward for 2 seconds")
                            robot.move_forward()
                            
                main.last_movement_time = current_time  # Update last movement time
            
            # Display timing information
            time_until_next = max(0, 2.5 - (current_time - main.last_movement_time))
            cv2.putText(frame, f"Next move in: {time_until_next:.1f}s", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Display the frame
            cv2.imshow('Robot Vision', frame)

            # Break loop on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")

    finally:
        # Clean up
        robot.stop()
        robot.serial_conn.close()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()