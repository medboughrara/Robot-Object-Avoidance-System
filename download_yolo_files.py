import urllib.request
import os
import ssl

def download_file(url, filename):
    print(f"Downloading {filename}...")
    # Create SSL context that ignores certificate validation
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    
    try:
        with urllib.request.urlopen(url, context=ctx) as response:
            with open(filename, 'wb') as f:
                f.write(response.read())
        print(f"Successfully downloaded {filename}")
    except Exception as e:
        print(f"Error downloading {filename}: {str(e)}")

# Create the yolo_files directory if it doesn't exist
yolo_dir = 'yolo_files'
if not os.path.exists(yolo_dir):
    os.makedirs(yolo_dir)

# Download configuration and names files
files = {
    'yolov3.cfg': 'https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov3.cfg',
    'coco.names': 'https://raw.githubusercontent.com/AlexeyAB/darknet/master/data/coco.names'
}

for filename, url in files.items():
    filepath = os.path.join(yolo_dir, filename)
    if not os.path.exists(filepath):
        download_file(url, filepath)
    else:
        print(f"{filename} already exists")

print("\nIMPORTANT: You need to manually download YOLOv3 weights:")
print("1. Go to: https://pjreddie.com/media/files/yolov3.weights")
print("2. Download the file (236MB)")
print("3. Move the downloaded 'yolov3.weights' file to the 'yolo_files' folder")
print(f"4. The final path should be: {os.path.join(yolo_dir, 'yolov3.weights')}")