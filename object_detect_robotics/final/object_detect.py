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

def object_detection(frame, net, layers_names, confidence_threshold=0.5):
    height, width, _ = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(layers_names)

    detected_objects = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > confidence_threshold:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                detected_objects.append((classes[class_id], confidence, (x, y, w, h)))

    return detected_objects

def main():
    # Load YOLO model
    net, classes, layers_names = load_yolo()

    # Open a connection to the webcam (0 represents the default camera)
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Perform object detection
        detected_objects = object_detection(frame, net, layers_names)

        # Draw bounding boxes and display results
        for obj in detected_objects:
            label, confidence, box = obj
            x, y, w, h = box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"{label}: {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow("Object Detection", frame)

        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
