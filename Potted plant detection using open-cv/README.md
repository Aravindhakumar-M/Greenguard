# YOLOv3 Tiny Object Detection with OpenCV

This Python script demonstrates object detection using the YOLOv3 Tiny model with OpenCV. It loads a pre-trained YOLOv3 Tiny model, processes an image, and detects objects within it. Detected objects are annotated with bounding boxes and labels.

## Prerequisites

Before running this script, ensure you have the following:

- Python 3
- OpenCV (cv2) installed
- A YOLOv3 Tiny model configuration file (`yolov3-tiny.cfg`)
- YOLOv3 Tiny model weights file (`yolov3-tiny.weights`)
- A class names file (`obj.names`)
- An image for object detection (e.g., `img.jpg`)

## Configuration

The script may require configuration based on your hardware setup:

- Update YOLOv3 Tiny model file paths (`yolov3-tiny.cfg` and `yolov3-tiny.weights`).
- Adjust the input image file path.
- Modify class names in the `obj.names` file as needed.

## Usage

1. Ensure you have OpenCV and the necessary YOLOv3 Tiny files (configuration and weights) in the specified paths.

2. Run the Python script using the following command:

```bash
python3 pottedplant_detecter.py
```

1. The script loads the YOLOv3 Tiny model, reads the image, and performs object detection.
2. Detected objects are annotated with bounding boxes and labels.
3. The script prints information about the detected objects, including the class and confidence score.
4. The annotated image with detected objects is displayed using Matplotlib.

## Node Information
Node Name:  
`yolo_object_detection`  

Outputs:  
Detected object information, including class and confidence score.  
Annotated image with bounding boxes and labels.

##  Acknowledgments
This script demonstrates object detection using the YOLOv3 Tiny model with OpenCV. Credit to the creators of YOLO (You Only Look Once) for the model architecture and the OpenCV community. Extending gratitude to Anya Robotics Pvt Ltd for their valuable contributions and support.
