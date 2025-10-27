# Robot Object Avoidance System - Raspberry Pi Setup Guide

This guide will help you set up the object avoidance robot system on a Raspberry Pi (2GB RAM) from scratch.

## Hardware Requirements

- Raspberry Pi (2GB RAM)
- MicroSD card (minimum 32GB recommended)
- Arduino Uno
- L298N Motor Driver
- DC Motors (x2)
- USB Camera or Raspberry Pi Camera
- HC-SR04 Ultrasonic Distance Sensor
- USB cable (for Arduino connection)
- Power supply for Raspberry Pi (5V/3A recommended)
- Power supply for motors (6-12V depending on your motors)
- Jumper wires for ultrasonic sensor connection

## 1. Initial Raspberry Pi Setup

### 1.1 Operating System Installation

1. Download "Raspberry Pi OS Lite (64-bit)" from the official website
   ```
   https://www.raspberrypi.com/software/operating-systems/
   ```
   Note: We use the Lite version to save RAM for our object detection system.

2. Flash the OS using Raspberry Pi Imager
   - Download from: https://www.raspberrypi.com/software/
   - Select the downloaded OS
   - Select your SD card
   - In settings (gear icon):
     - Set hostname (e.g., robotpi)
     - Enable SSH
     - Set username and password
     - Configure WiFi (if needed)
   - Write to SD card

3. Insert the SD card into Raspberry Pi and power it on

### 1.2 Initial Configuration

1. Connect to your Raspberry Pi via SSH:
   ```bash
   ssh username@robotpi.local
   ```

2. Update the system:
   ```bash
   sudo apt update
   sudo apt upgrade -y
   ```

3. Install required system packages:
   ```bash
   sudo apt install -y python3-pip python3-opencv git arduino arduino-core
   sudo apt install -y libatlas-base-dev  # Required for numpy
   ```

## 2. Project Setup

### 2.1 Create Project Directory

```bash
mkdir ~/robot_project
cd ~/robot_project
```

### 2.2 Install Python Dependencies

1. Create and activate virtual environment:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

2. Install required Python packages:
   ```bash
   # Install basic requirements
   pip install opencv-python numpy pyserial Pillow
   
   # Install TensorFlow Lite Runtime (much smaller than full TensorFlow)
   pip install tflite-runtime
   
   # If tflite-runtime installation fails, try the alternative installation:
   echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
   curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
   sudo apt-get update
   sudo apt-get install python3-tflite-runtime
   
   # If Pillow installation fails, install system dependencies first:
   sudo apt-get install -y python3-pil python3-pil.imagetk
   pip install Pillow
   ```

### 2.3 Download Project Files

1. Clone the project repository or copy the files:
   ```bash
   # Copy these files to your project directory:
   # - object_avoid_robot.py
   # - robot_control.ino
   ```

2. Create YOLO files directory:
   ```bash
   mkdir yolo_files
   cd yolo_files
   ```

3. Download YOLO files:
   ```bash
   # Download configuration file
   wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov3.cfg
   
   # Download COCO names file
   wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/data/coco.names
   
   # Download YOLOv3 weights (237MB) - Method 1
   wget https://github.com/patrick013/Object-Detection---Yolov3/raw/master/yolov3.weights
   
   # If Method 1 fails, try Method 2:
   curl -L https://pjreddie.com/media/files/yolov3.weights -o yolov3.weights
   
   # If both methods fail, download manually from browser:
   # 1. Visit: https://pjreddie.com/darknet/yolo/
   # 2. Click on "YOLOv3-416" weights download link
   # 3. Transfer the file to your Raspberry Pi using SCP:
   #    On your local machine:
   #    scp yolov3.weights username@robotpi.local:~/robot_project/yolo_files/
   
   # Verify the download:
   # The file should be approximately 237MB
   ls -lh yolov3.weights
   # Should show around 237M
   ```
   
   Note: If you have trouble downloading the weights file, you can also try using a smaller model like YOLOv3-tiny:
   ```bash
   # Alternative: Download YOLOv3-tiny (34MB)
   wget https://pjreddie.com/media/files/yolov3-tiny.weights
   wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov3-tiny.cfg
   
   # Then update the paths in object_avoid_robot.py to use these files instead
   ```

## 3. Arduino Setup

1. Connect Arduino to Raspberry Pi via USB

2. Upload the Arduino code:
   ```bash
   # Install Arduino CLI if not installed
   curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
   
   # Compile and upload robot_control.ino
   arduino-cli compile --fqbn arduino:avr:uno robot_control
   arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno robot_control
   ```

## 4. Hardware Connections

### 4.1 Arduino Pin Connections

#### Arduino to L298N Motor Driver

| Arduino Pin | L298N Pin |
|------------|-----------|
| 6          | ENA       |
| 7          | IN1       |
| 8          | IN2       |
| 9          | ENB       |
| 10         | IN3       |
| 11         | IN4       |

#### Arduino to HC-SR04 Ultrasonic Sensor

| Arduino Pin | HC-SR04 Pin |
|------------|-------------|
| 5V         | VCC         |
| GND        | GND         |
| 4          | TRIG        |
| 5          | ECHO        |

Note: The ultrasonic sensor provides distance measurements for improved obstacle detection

### 4.2 Power Connections

1. Connect motor power supply (6-12V) to L298N
2. Connect motors to L298N output terminals
3. Ensure common ground between Arduino and L298N

## 5. Running the System

1. Activate the virtual environment:
   ```bash
   cd ~/robot_project
   source venv/bin/activate
   ```

2. Run the object avoidance system:
   ```bash
   python object_avoid_robot.py --serial-port /dev/ttyACM0
   ```

### 5.1 Movement Control System

The robot uses a hybrid control system combining visual detection and ultrasonic distance sensing:

1. Timed Movement System:
   - Each movement command executes for exactly 2 seconds
   - Robot automatically stops after each movement
   - 0.5-second pause between movements
   - Visual countdown timer shown on video feed

2. Safety Features:
   - Ultrasonic sensor provides real-time distance measurements
   - Emergency stop triggered if distance < 20cm
   - Warning mode activated if distance < 40cm
   - Emergency stops override timed movement system

3. Movement Thresholds:
   - Emergency Stop: < 20cm (immediate backward movement)
   - Warning Zone: 20-40cm (careful navigation)
   - Safe Zone: > 40cm (normal operation)

   Options:
   - `--camera`: Specify camera device number (default: 0)
     Examples:
     ```bash
     # Try different video devices if the default doesn't work:
     python object_avoid_robot.py --camera 0  # First camera
     python object_avoid_robot.py --camera 2  # Third camera
     
     # If you have identified the correct device, you can specify it directly:
     python object_avoid_robot.py --camera $(readlink -f /dev/video0 | grep -o "[0-9]*$")
     ```
   - `--serial-port`: Specify Arduino port (usually /dev/ttyACM0)
   
   Note: When multiple video devices are present, you may need to try different
   numbers to find your camera. Use the troubleshooting steps in the section
   below to identify the correct camera device.

## Real-Time Performance Version

For better real-time performance on mobile robots, use the optimized version:

1. Download YOLOv3-tiny files:
   ```bash
   cd ~/robot_project/yolo_files
   wget https://pjreddie.com/media/files/yolov3-tiny.weights
   wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov3-tiny.cfg
   ```

2. Run the real-time version:
   ```bash
   python real_time_robot.py --serial-port /dev/ttyACM0
   ```

   Additional options:
   - `--confidence 0.4`: Adjust detection confidence (default: 0.5)
   - `--camera 0`: Select camera device
   
   The real-time version includes:
   - Smaller model (YOLOv3-tiny)
   - Optimized processing
   - FPS counter
   - Reduced latency
   - Lower resource usage

## Performance Optimization for 2GB RAM

1. Close unnecessary programs and services:
   ```bash
   sudo systemctl disable bluetooth
   sudo systemctl disable cups
   ```

2. Add swap space:
   ```bash
   sudo fallocate -l 4G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

3. Add to /etc/fstab for permanent swap:
   ```bash
   sudo echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
   ```

## Troubleshooting

1. If ultrasonic sensor readings are incorrect:
   ```bash
   # Check sensor connections
   - Verify all four pins are properly connected
   - Check for loose wires or poor connections
   - Ensure TRIG and ECHO pins match Arduino code
   
   # Test sensor independently
   - Use Arduino IDE''s serial monitor
   - Look for consistent distance readings
   - Normal readings should be between 2-400cm
   
   # Common issues:
   - Erratic readings: Check power supply and connections
   - Always 0cm: Check TRIG/ECHO pin connections
   - Maximum distance: Sensor might be blocked or misaligned
   ```

2. If timed movements aren't working:
   ```bash
   # Verify movement timing
   - Watch the countdown timer on video feed
   - Each movement should last exactly 2 seconds
   - There should be a 0.5s pause between movements
   
   # Check Arduino responses
   - Look for "OK:" prefix in responses
   - Verify stop commands are being received
   - Check for any error messages
   ```

3. If camera doesn't work:
   ```bash
   # List all video devices
   ls /dev/video*
   
   # If you see multiple devices (like video0, video1, etc.), identify the correct one:
   v4l2-ctl --list-devices
   
   # Get detailed information about a specific camera:
   v4l2-ctl --device=/dev/video0 --all
   
   # Test camera preview (replace video0 with your camera device):
   ffplay /dev/video0
   
   # Common video device patterns:
   # - video0: Usually the first USB camera or built-in camera
   # - video2/video3: Sometimes used by Raspberry Pi Camera Module
   # - Higher numbers (video10+): Often virtual devices or camera subdevices
   
   # If needed, install v4l-utils for camera testing:
   sudo apt install v4l-utils ffmpeg
   ```
   
   Note: If using Raspberry Pi Camera Module, enable it first:
   ```bash
   sudo raspi-config
   # Navigate to: Interface Options > Camera > Enable
   # Reboot after enabling
   ```

2. If Arduino connection fails:
   ```bash
   # On Linux (Raspberry Pi):
   # List all USB serial devices
   ls /dev/tty*
   
   # Common Arduino ports:
   ls /dev/ttyACM*  # Arduino Uno, Mega
   ls /dev/ttyUSB*  # Arduino with CH340/CP2102 chips
   
   # Add user to dialout group (needed for serial access)
   sudo usermod -a -G dialout $USER
   sudo chmod a+rw /dev/ttyACM0  # Replace with your port
   
   # Log out and log back in for group changes to take effect
   # Or temporarily:
   newgrp dialout
   
   # On Windows:
   # Check Device Manager > Ports (COM & LPT)
   # Or use PowerShell:
   [System.IO.Ports.SerialPort]::getportnames()
   
   # Test Arduino connection:
   # Install screen to test serial connection
   sudo apt install screen
   
   # Test connection (replace with your port)
   screen /dev/ttyACM0 9600
   # Press Ctrl-A, then k to exit screen
   
   # When running the program, specify the correct port:
   python object_avoid_robot.py --camera 0 --serial-port /dev/ttyUSB0  # Linux example
   python object_avoid_robot.py --camera 0 --serial-port COM3  # Windows example
   ```
   
   Note: If the Arduino is not recognized:
   1. Try a different USB cable
   2. Try a different USB port
   3. Check if the Arduino is detected:
   ```bash
   dmesg | grep -i "tty"
   lsusb  # List USB devices
   ```

3. If system is slow:
   - Reduce camera resolution in code
   - Adjust confidence threshold
   - Use smaller YOLO model (like YOLOv3-tiny)

## Safety Notes

- Always test the robot in a safe environment
- Keep emergency stop button accessible
- Monitor system temperature
- Ensure proper voltage for motors
- Verify ultrasonic sensor connections and readings before operation
- Test emergency stop system before each use
- Keep objects at least 50cm away during initial testing
- Monitor both camera feed and distance readings during operation
- Ensure clear line of sight for both camera and ultrasonic sensor

## Additional Tips

1. For autostart on boot:
   ```bash
   sudo nano /etc/rc.local
   # Add before exit 0:
   cd /home/username/robot_project && source venv/bin/activate && python object_avoid_robot.py &
   ```

2. Monitor system resources:
   ```bash
   htop  # Install with: sudo apt install htop
   ```

3. Check logs:
   ```bash
   tail -f /var/log/syslog  # System logs
   dmesg | tail  # Kernel messages
   ```