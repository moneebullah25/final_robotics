import os
import cv2
import numpy as np

# Assuming you have YOLO-related files in the same directory as this script
yolo_config_path = "yolov3.cfg"
yolo_weights_path = "yolov3.weights"
coco_names_path = "coco.names"

def load_yolo():
    net = cv2.dnn.readNet(yolo_weights_path, yolo_config_path)
    classes = []
    with open(coco_names_path, "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layers_names = net.getUnconnectedOutLayersNames()
    return net, classes, layers_names

def load_images_and_labels_from_folders(folder):
    images = []
    labels = []
    for filename in os.listdir(folder):
        img_path = os.path.join(folder, filename)
        if os.path.isfile(img_path):
            images.append(img_path)
            labels.append(os.path.basename(folder))
    return images, labels

def main():
    # Load YOLO model
    net, classes, layers_names = load_yolo()

    # Load images and labels from folders
    robot_images, robot_labels = load_images_and_labels_from_folders("path/to/Robot")
    target_images, target_labels = load_images_and_labels_from_folders("path/to/Target")

    all_images = robot_images + target_images
    all_labels = robot_labels + target_labels

    # Training YOLO on the loaded images
    for img_path, label in zip(all_images, all_labels):
        image = cv2.imread(img_path)
        height, width, _ = image.shape
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(layers_names)

    # Save the trained model locally
    net.save("trained_model_yolo.weights")

if __name__ == "__main__":
    main()
