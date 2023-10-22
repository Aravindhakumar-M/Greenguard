import cv2
import numpy as np
import matplotlib.pyplot as plt

# Define confidence and non-maximum suppression thresholds
CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.5

# Define colors for bounding boxes
COLORS = [(0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]

# Define font for text overlay on the image
fonts = cv2.FONT_HERSHEY_COMPLEX

# Load YOLOv3 Tiny model and set preferences
net = cv2.dnn.readNet('yolov3-tiny.weights', 'yolov3-tiny.cfg', 'DNN_TARGET_MYRIAD')
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

# Create a detection model using the loaded YOLOv3 Tiny network
model = cv2.dnn_DetectionModel(net)
model.setInputParams(size=(416, 416), scale=0.00392, swapRB=True)

# Load an image for object detection
image = cv2.imread("img.jpg")
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Read class names from obj.names file
class_names = []
with open("obj.names", "r") as f:
    class_names = [line.strip() for line in f.readlines()]

# Check if the image is loaded successfully
if image is not None:
    width, height = 416, 416  # YOLOv3 input size
    image = cv2.resize(image, (width, height))
    image = image / 255.0

    image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # Create a blob from the preprocessed image
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (width, height), swapRB=True, crop=False)
    
    # Set the input blob for the network
    net.setInput(blob)
    
    # Get layer names and output layer names
    layer_names = net.getLayerNames()
    output_layer_names = net.getUnconnectedOutLayersNames()
    
    # Forward pass through the network
    outs = net.forward(output_layer_names)

    # Detect objects in the image
    classes, scores, boxes = model.detect(image, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
    
    # Process detected objects
    for (classid, score, box) in zip(classes, scores, boxes):
        if class_names[0] == 'pottedplant':
            print("Detected Plant pot!!!")
            color = COLORS[int(classid) % len(COLORS)]
            label = "%s : %f" % (class_names[0], score)
            
            # Draw bounding box and label on the image
            cv2.rectangle(image, box, color, 2)
            cv2.putText(image, label, (box[0], box[1]-10), fonts, 0.5, color, 2)

            # Find the position of the plant pot
            object_center_x = (box[0] + box[2] // 2)
            frame_part = 416 // 21
            position = ['-10', '-9', '-8', '-7', '-6', '-5', '-4', '-3', '-2', '-1', '0', '+1', '+2', '+3', '+4', '+5', '+6', '+7', '+8', '+9', '+10']
            position_output = '0'
            
            # Determine the position based on the object's center
            for i in range(1, 11):
                if object_center_x < frame_part * i:
                    position_output = position[i-1]
            for i in range(20, 11, -1):
                if object_center_x > frame_part * i:
                    position_output = position[i]
            
            print("Position: ", position_output)

    # Display the image with detected objects
    plt.imshow(image)
else:
    print("Failed to load the image.")
