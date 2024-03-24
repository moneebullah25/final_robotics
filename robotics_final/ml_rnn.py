import cv2
import numpy as np
import time
import socket

TCP_IP = '192.168.93.215' # put esp32 on serial monitor and copy the ip from there
TCP_PORT = 10000
BUFFER_SIZE = 1024

connected = False

while not connected:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((TCP_IP, TCP_PORT))
        connected = True
        print("Connection successful!")
    except Exception as e:
        print(f"Connection failed: {e}")
        time.sleep(1)

print(f"Connected to {TCP_IP} on PORT {TCP_PORT}")

left_motor = '100'
right_motor = '100'

# Constants
NUM_ACTIONS = 4  # move left, move forward, move right, and stop
ACTIONS = ['LEFT', 'FORWARD', 'RIGHT', 'STOP']
LEARNING_RATE = 0.1
DISCOUNT_FACTOR = 0.9
EPSILON = 0.1  # Exploration-exploitation trade-off

# Initialize Q-table
bins = range(0, 640, 20)
state_space_size = len(bins)  # Since we have 7 dicreate distances
action_space_size = NUM_ACTIONS
q_table = np.zeros((state_space_size, action_space_size))

# Initialize lists to store coordinates
green_coords = []
red_coords = []
grey_coords = []

GREEN_RECT = 100
RED_RECT = 20
GRAY_RECT = 200

# Timing control
start_time = time.time()

# Main loop
green_cord = [-1, -1]
red_cord = [-1, -1]
gray_cord = [-1,-1]

# Function to discretize the distance for Q-table indexing
def discretize_distance(distance):
    # You might need to adjust the bin sizes based on your specific scenario
    return np.digitize(distance, bins) - 1

# Q-learning algorithm
def q_learning(state, action, reward, next_state):
    current_q = q_table[state, action]
    best_future_q = np.max(q_table[next_state, :])
    new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * (reward + DISCOUNT_FACTOR * best_future_q)
    q_table[state, action] = new_q

def reward_func(dist: float, nextDist: float):
    obstacle_dist = (((gray_cord[0] - red_cord[0]) ** 2) + ((gray_cord[1] - red_cord[1]) ** 2)) ** 0.5
    if obstacle_dist < 100:
        return -1 * (dist - nextDist)
    if obstacle_dist > 100:
        return 1 * (dist - nextDist)
    return (dist - nextDist)

def nothing(x):
    pass

def moveForward(s: socket):
    values = left_motor + ',' + right_motor
    print("Move Forward", values)
    s.send((values + '\n').encode())
    time.sleep(0.3)

def moveLeft(s: socket):
    values = left_motor + ',' + '0'
    print("Move Right", values)
    s.send((values + '\n').encode())
    time.sleep(0.3)

def moveRight(s: socket):
    values = '0' + ',' + right_motor
    s.send((values + '\n').encode())
    time.sleep(0.3)

def stop(s: socket):
    values = "0,0"
    print("Stop", values)
    s.send((values + '\n').encode())
    time.sleep(0.3)

# Function to calculate average of coordinates
def calculate_average(coords):
    if coords:
        avg = np.mean(coords, axis=0)
        return int(avg[0]), int(avg[1])
    return None

# Function to draw coordinates on frame
def draw_coordinates(frame, coords, color, label):
    if coords:
        cv2.putText(frame, f"{label}: {coords}", (10, 30 if label == 'Green' else 60 if label == 'Red' else 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

# Function to draw rectangles and lines on the frame
def process_frame(frame, green_mask, red_mask, gray_mask):
    # Find contours for the green patch and draw bounding box
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in green_contours:
        area = cv2.contourArea(contour)
        if area > GREEN_RECT:  # Adjust the area threshold as needed
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            green_cord[0] = x + w
            green_cord[1] = y + h
            

    # Find contours for the Red patch and draw bounding box
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in red_contours:
        area = cv2.contourArea(contour)
        if area > RED_RECT:  # Adjust the area threshold as needed
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            red_cord[0] = x + w
            red_cord[1] = y + h

    # Find contours for the gray objects and draw bounding boxes
    gray_contours, _ = cv2.findContours(gray_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in gray_contours:
        area = cv2.contourArea(contour)
        if area > GRAY_RECT:  # Adjust the area threshold as needed
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (128, 128, 128), 2)  # Use gray color (128, 128, 128)
            gray_cord[0] = x + w
            gray_cord[1] = y + h
    
    green_central = None
    red_central = None
    gray_central = None
    
    # Processing grey contours
    if len(gray_contours) > 0:
        largest_contour = max(gray_contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > GRAY_RECT:  # Adjust the area threshold as needed
            x, y, w, h = cv2.boundingRect(largest_contour)
            gray_central = (int(x + w / 2), int(y + h / 2))
            cv2.circle(frame, gray_central, 5, (128, 128, 128), -1)
    
    # Processing green contours
    if len(green_contours) > 0:
        largest_contour = max(green_contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > GREEN_RECT:  # Adjust the area threshold as needed
            x, y, w, h = cv2.boundingRect(largest_contour)
            green_central = (int(x + w / 2), int(y + h / 2))
            cv2.circle(frame, green_central, 5, (0, 255, 0), -1)

    # Processing Red contours
    if len(red_contours) > 0:
        largest_contour = max(red_contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > RED_RECT:  # Adjust the area threshold as needed
            x, y, w, h = cv2.boundingRect(largest_contour)
            red_central = (int(x + w / 2), int(y + h / 2))
            cv2.circle(frame, red_central, 5, (0, 0, 255), -1)
    
    return green_central, red_central, gray_central

def get_masks(frame_hsv):
    # Get current positions of the green trackbars
    low_h_gray = cv2.getTrackbarPos('Gray Low H', 'HSV Adjustments')
    high_h_gray = cv2.getTrackbarPos('Gray High H', 'HSV Adjustments')
    low_s_gray = cv2.getTrackbarPos('Gray Low S', 'HSV Adjustments')
    high_s_gray = cv2.getTrackbarPos('Gray High S', 'HSV Adjustments')
    low_v_gray = cv2.getTrackbarPos('Gray Low V', 'HSV Adjustments')
    high_v_gray = cv2.getTrackbarPos('Gray High V', 'HSV Adjustments')

    low_h_Red = cv2.getTrackbarPos('Red Low H', 'HSV Adjustments')
    high_h_Red = cv2.getTrackbarPos('Red High H', 'HSV Adjustments')
    low_s_Red = cv2.getTrackbarPos('Red Low S', 'HSV Adjustments')
    high_s_Red = cv2.getTrackbarPos('Red High S', 'HSV Adjustments')
    low_v_Red = cv2.getTrackbarPos('Red Low V', 'HSV Adjustments')
    high_v_Red = cv2.getTrackbarPos('Red High V', 'HSV Adjustments')

    low_h_green = cv2.getTrackbarPos('Green Low H', 'HSV Adjustments')
    high_h_green = cv2.getTrackbarPos('Green High H', 'HSV Adjustments')
    low_s_green = cv2.getTrackbarPos('Green Low S', 'HSV Adjustments')
    high_s_green = cv2.getTrackbarPos('Green High S', 'HSV Adjustments')
    low_v_green = cv2.getTrackbarPos('Green Low V', 'HSV Adjustments')
    high_v_green = cv2.getTrackbarPos('Green High V', 'HSV Adjustments')

    # Define gray color range with trackbar values
    lower_gray = np.array([low_h_gray, low_s_gray, low_v_gray])
    upper_gray = np.array([high_h_gray, high_s_gray, high_v_gray])


    # Define green color range with trackbar values
    lower_green = np.array([low_h_green, low_s_green, low_v_green])
    upper_green = np.array([high_h_green, high_s_green, high_v_green])

    # Define Red color range with trackbar values
    lower_Red = np.array([low_h_Red, low_s_Red, low_v_Red])
    upper_Red = np.array([high_h_Red, high_s_Red, high_v_Red])

    # Create masks to detect color patches
    green_mask = cv2.inRange(frame_hsv, lower_green, upper_green)
    red_mask = cv2.inRange(frame_hsv, lower_Red, upper_Red)
    gray_mask = cv2.inRange(frame_hsv, lower_gray, upper_gray)

    return green_mask, red_mask, gray_mask

# Initialize camera
cap = cv2.VideoCapture(1)

# Initialize window for trackbars
cv2.namedWindow('HSV Adjustments', cv2.WINDOW_NORMAL)

# Create trackbars for gray color detection
cv2.createTrackbar('Gray Low H', 'HSV Adjustments', 0, 179, nothing)
cv2.createTrackbar('Gray High H', 'HSV Adjustments', 52, 179, nothing)
cv2.createTrackbar('Gray Low S', 'HSV Adjustments', 0, 255, nothing)
cv2.createTrackbar('Gray High S', 'HSV Adjustments', 56, 255, nothing)
cv2.createTrackbar('Gray Low V', 'HSV Adjustments', 56, 255, nothing)
cv2.createTrackbar('Gray High V', 'HSV Adjustments', 108, 255, nothing)

# Create trackbars for red color detection
cv2.createTrackbar('Red Low H', 'HSV Adjustments', 160, 179, nothing)
cv2.createTrackbar('Red High H', 'HSV Adjustments', 179, 179, nothing)
cv2.createTrackbar('Red Low S', 'HSV Adjustments', 0, 255, nothing)
cv2.createTrackbar('Red High S', 'HSV Adjustments', 255, 255, nothing)
cv2.createTrackbar('Red Low V', 'HSV Adjustments', 59, 255, nothing)
cv2.createTrackbar('Red High V', 'HSV Adjustments', 255, 255, nothing)

# Create trackbars for green color detection
cv2.createTrackbar('Green Low H', 'HSV Adjustments', 81, 179, nothing)
cv2.createTrackbar('Green High H', 'HSV Adjustments', 94, 179, nothing)
cv2.createTrackbar('Green Low S', 'HSV Adjustments', 40, 255, nothing)
cv2.createTrackbar('Green High S', 'HSV Adjustments', 255, 255, nothing)
cv2.createTrackbar('Green Low V', 'HSV Adjustments', 40, 255, nothing)
cv2.createTrackbar('Green High V', 'HSV Adjustments', 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get masks from the green, red and gray shades define
    green_mask, red_mask, gray_mask = get_masks(frame_hsv) 

    # Get the center point for red and green and draw the circle against the detected objects
    green_central, red_central, gray_central = process_frame(frame, green_mask, red_mask, gray_mask)

    # Draw a line between green and Red central points if both are detected
    if green_central and red_central:
        cv2.line(frame, green_central, red_central, (255, 0, 0), 2)
        # Calculate distance
        dist = np.linalg.norm(np.array(green_central) - np.array(red_central))

        # Discretize distance for indexing the Q-table
        state = discretize_distance(dist)

        # Choose action using epsilon-greedy strategy
        if np.random.rand() < EPSILON:
            action = np.random.choice(NUM_ACTIONS)
        else:
            action = np.argmax(q_table[state, :])
        
        # Send the chosen action to your robot (moveLeft, moveForward, moveRight, stop)
        
        if action == 0:
            moveLeft(s)
        elif action == 1:
            moveForward(s)
        elif action == 2:
            moveRight(s)
        elif action == 3:
            stop(s)

        # Again read the coordinates after performing the action
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get masks from the green, red and gray shades define
        green_mask, red_mask, gray_mask = get_masks(frame_hsv) 

        # Get the center point for red and green and draw the circle against the detected objects
        green_central, red_central, gray_central = process_frame(frame, green_mask, red_mask, gray_mask)

        # If not green and red points are not found after taking action move to next iteration
        if not green_central or not red_central:
            continue
        
        cv2.line(frame, green_central, red_central, (255, 0, 0), 2)
        next_dist = np.linalg.norm(np.array(green_central) - np.array(red_central))
        next_state = discretize_distance(next_dist)

        # Calculate reward based on your reward function
        reward = reward_func(state, next_state)

        # Update Q-value
        q_learning(state, action, reward, next_state)

        print(f"Distance: {dist} Green Central {green_central} Red Central {red_central} Frame Shape {frame.shape}")
        print(f"Distance {dist} over {bins[state]} Action {ACTIONS[action]}")
        print(f"Next Distance {next_dist} over {bins[discretize_distance(next_dist)]} Next State {bins[state]} Reward {reward}")
    
    
    if green_coords:
        draw_coordinates(frame, green_coords[-1], (0, 255, 0), 'Green')
    if red_coords:
        draw_coordinates(frame, red_coords[-1], (0, 0, 255), 'Red')
    if grey_coords:
        draw_coordinates(frame, grey_coords[-1], (128, 128, 128), 'Grey')    
    
    # Display the frame
    cv2.imshow('Frame', frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release camera and close all windows
cap.release()
cv2.destroyAllWindows()

print("Disconnected")
# s.close()