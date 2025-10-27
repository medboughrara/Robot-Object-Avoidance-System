import cv2
import numpy as np
import serial
import time
import os
import argparse
class RobotController:
    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=9600):
        """Initialize the robot controller with serial connection to Arduino"""
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to Arduino on {serial_port}")
        except Exception as e:
            print(f"Error connecting to Arduino: {e}")
            print("If you're on Windows, try using 'COM3' (or another COM port)")
            print("If you're on Linux, try using '/dev/ttyACM0' or '/dev/ttyUSB0'")
            exit(1)

    def send_command(self, command):
        """Send a command to the Arduino"""
        try:
            self.serial_conn.write(command.encode())
            time.sleep(0.1)  # Give Arduino time to process
            response = self.serial_conn.readline().decode().strip()
            print(f"Arduino response: {response}")
        except Exception as e:
            print(f"Error sending command: {e}")

    def move_forward(self):
        self.send_command("F\n")  # Forward

    def move_backward(self):
        self.send_command("B\n")  # Backward

    def turn_left(self):
        self.send_command("L\n")  # Left

    def turn_right(self):
        self.send_command("R\n")  # Right

    def stop(self):
        self.send_command("S\n")  # Stop
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
def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Robot Object Avoidance')
    parser.add_argument('--camera', type=int, default=0,
                        help='Camera index (default: 0)')
    parser.add_argument('--serial-port', type=str,
                        default='/dev/ttyACM0' if os.name != 'nt' else 'COM3',
                        help='Serial port for Arduino')
    args = parser.parse_args()

    # Initialize robot controller and object detector
    robot = RobotController(serial_port=args.serial_port)
    detector = ObjectDetector()

    # Initialize camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Error: Could not open camera {args.camera}")
        exit(1)

    print("Starting object avoidance system...")
    print("Press 'q' to quit")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Can't receive frame from camera")
                break

            # Detect objects
            frame, detections = detector.detect_objects(frame)
            
            # Process detections and control robot
            if detections:
                # Get the closest/largest object
                frame_center_x = frame.shape[1] / 2
                frame_center_y = frame.shape[0] / 2
                
                # Get the first detection (usually the most prominent)
                detection = detections[0]
                object_center_x = detection['center'][0]
                object_center_y = detection['center'][1]
                
                # Calculate distance from center (simple logic)
                distance_x = object_center_x - frame_center_x
                distance_y = object_center_y - frame_center_y
                
                # Implement avoidance logic
                if abs(distance_x) > 100:  # Object is significantly off-center
                    if distance_x < 0:
                        print("Object on left - Moving right")
                        robot.turn_right()
                    else:
                        print("Object on left - Moving left")
                        robot.turn_left()
                else:
                    # If object is too close (based on box height)
                    if detection['box'][3] > frame.shape[0] * 0.4:  # If object takes up 40% of frame height
                        print("Object too close - Moving backward")
                        robot.move_backward()
                    else:
                        print("Path clear - Moving forward")
                        robot.move_forward()
            else:
                # No objects detected, move forward
                print("No objects detected - Moving forward")
                robot.move_forward()

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
