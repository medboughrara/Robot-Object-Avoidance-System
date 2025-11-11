# YOLO + SAM Robot Controller Setup Guide

## Overview
This advanced version uses:
- **YOLOv3** for real-time object detection
- **SAM (Segment Anything Model)** for precise object segmentation
- **Arduino** for robot movement control
- **GUI Interface** for easy camera selection and control

## Installation

### 1. Basic Requirements
```bash
pip install -r requirements_yolo_sam.txt
```

### 2. Download YOLO Files
Make sure you have the following files in the `yolo_files/` directory:
- `yolov3.cfg` - YOLO configuration file
- `yolov3.weights` - YOLO pre-trained weights (237MB)
- `coco.names` - Class labels

You should already have these files since they were uploaded with the project.

### 3. Optional: Install SAM (Segment Anything Model)
SAM provides more accurate object segmentation. It's optional but recommended.

```bash
# Install SAM from Facebook Research
pip install git+https://github.com/facebookresearch/segment-anything.git

# Download the SAM checkpoint (one of the following)
# ViT-B (small): ~375MB - Recommended for faster inference
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth

# ViT-L (large): ~1.2GB - More accurate but slower
# wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_l_0b3195.pth

# ViT-H (huge): ~2.6GB - Most accurate but very slow
# wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth

# Place the checkpoint in the project root directory
```

## Running the Application

### Basic Usage
```bash
python yolo_sam_robot_controller.py
```

### Features

#### 🎥 Camera Selection
1. Click on a camera from the list
2. Click "Connect" to open the selected camera
3. Live YOLO detection will display in the video panel

#### 🤖 Arduino Control

**Manual Control:**
- Use arrow buttons to control robot movement
- Forward, Backward, Left, Right, Stop

**Automatic Control:**
- Enable "Auto Control" checkbox
- Robot will automatically avoid detected objects based on:
  - Object position in frame
  - Object size (proximity)
  - Ultrasonic distance sensor (if connected)

#### 🎯 Detection Logic

The robot makes decisions based on:

1. **Emergency Stop (< 20cm)**: 
   - If ultrasonic sensor detects object within 20cm
   - Robot moves BACKWARD immediately

2. **Warning Zone (20-40cm)**:
   - If distance sensor shows 20-40cm
   - Robot turns away from detected objects

3. **Size-Based Detection**:
   - If detected object occupies >40% of frame width/height
   - Robot moves BACKWARD (object too close)

4. **Position-Based Navigation**:
   - If object is off-center by >120 pixels
   - Robot turns LEFT or RIGHT to avoid it

5. **Clear Path**:
   - If no large objects detected and distance is safe
   - Robot moves FORWARD

#### 📊 Real-Time Display

The GUI shows:
- **Status Log**: Timestamped actions and events
- **Detection Count**: Number of detected objects
- **Distance**: Real-time ultrasonic sensor reading (if available)
- **Current Action**: What the robot is currently doing
- **Video Feed**: YOLO detections with bounding boxes and SAM segmentation masks (if available)

## Hardware Requirements

### Arduino Connection
- Arduino Uno connected via USB
- L298N Motor Driver
- 2x DC Motors
- HC-SR04 Ultrasonic Sensor (optional but recommended)

### Serial Port Configuration
- Windows: COM1, COM2, COM3, COM4, etc.
- Linux/Raspberry Pi: /dev/ttyACM0, /dev/ttyUSB0

## Troubleshooting

### YOLO Model Not Loading
```
Error: Required file not found: yolov3.weights
```
**Solution**: Download the yolov3.weights file from:
https://pjreddie.com/media/files/yolov3.weights

### SAM Not Available
```
Could not load SAM: No module named 'segment_anything'
```
**Solution**: The system will continue using YOLO only. To enable SAM:
```bash
pip install git+https://github.com/facebookresearch/segment-anything.git
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth
```

### Arduino Not Connecting
```
Warning: Could not connect to Arduino
```
**Solution**: 
1. Check USB cable connection
2. Verify Arduino port in the GUI dropdown
3. Check Device Manager (Windows) or `ls /dev/tty*` (Linux) to find the correct port
4. The system will work in simulation mode without Arduino

### Camera Not Opening
```
Error: Could not open camera
```
**Solution**:
1. Click "Stop" first
2. Ensure camera is not in use by another application
3. Try a different camera index from the list
4. Reconnect the camera USB cable

## Performance Optimization

### For Slow Systems:
1. Use **YOLOv3-tiny** instead of full YOLOv3
2. Disable SAM segmentation
3. Reduce camera resolution
4. Lower detection confidence threshold

### For Fast Inference:
1. Use GPU (CUDA) if available
2. Use ViT-B version of SAM
3. Increase movement interval for less frequent decisions

## Arduino Code Reference

Expected Arduino commands:
- `F\n` - Move Forward (for 2 seconds)
- `B\n` - Move Backward (for 2 seconds)
- `L\n` - Turn Left (for 2 seconds)
- `R\n` - Turn Right (for 2 seconds)
- `S\n` - Stop

Expected Arduino responses:
- `OK:` - Command executed successfully
- `D:XX.X` - Distance reading (XX.X cm)

## Safety Notes

⚠️ **IMPORTANT**:
1. Always test in a safe, enclosed area
2. Keep emergency stop accessible
3. Monitor the distance sensor readings
4. Test each movement manually first
5. Never leave the robot running unattended
6. Keep objects at least 50cm away during initial testing

## Files Included

- `yolo_sam_robot_controller.py` - Main GUI application
- `yolo_files/` - YOLO model files
- `requirements_yolo_sam.txt` - Python dependencies

## Contact & Support

For issues or improvements, please check:
- GitHub Issues
- Documentation in README.md
- Serial monitor output for Arduino debugging
