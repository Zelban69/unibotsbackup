import cv2
import numpy as np

# Function to send command to the robot (placeholder)
def send_robot_command(direction):
    print(f"Robot moving {direction}")  # Replace this print statement with actual code to control the robot.

# Set up the webcam
cap = cv2.VideoCapture(0)  # '0' is usually the default ID for the first connected camera
_, frame = cap.read()  # Read one frame to get the size

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

# Determine the center of the frame
frame_center_x = frame.shape[1] // 2
forward_area_threshold = 5000  # Adjust this threshold based on your testing
horizontal_threshold = 50  # Adjust this threshold based on your testing

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the range for white color
    lower_white = np.array([0, 0, 150], dtype=np.uint8)
    upper_white = np.array([180, 50, 255], dtype=np.uint8)

    # Create a mask for white color
    white_mask = cv2.inRange(hsv_frame, lower_white, upper_white)

    # Find contours in the mask
    contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Variables to keep track of the largest contour
    max_area = 0
    largest_contour = None

    # Loop through contours to find the largest one
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        # Avoid division by zero
        if perimeter == 0:
            continue

        # Check circularity to identify potential ping pong balls
        circularity = (4 * np.pi * area) / (perimeter ** 2)

        # Update the largest contour if this contour is larger than the previous largest
        if 0.3 < circularity <= 1.0 and area > max_area:  # im going to rip my hair out trying to adjust these values
            max_area = area
            largest_contour = contour

    # If a largest suitable contour has been found, annotate it
    if largest_contour is not None:
        (cx, cy), radius = cv2.minEnclosingCircle(largest_contour)
        center = (int(cx), int(cy))
        radius = int(radius)
        cv2.circle(frame, center, radius, (0, 255, 0), 3)
        cv2.putText(frame, 'Closest Ping Pong Ball', (center[0] - radius, center[1] - radius), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Determine the direction for the robot based on ball position and size
        if max_area < forward_area_threshold:
            send_robot_command("forward_move")
        elif cx > frame_center_x and abs(cx - frame_center_x) > horizontal_threshold:
            send_robot_command("right_move")
        elif cx < frame_center_x and abs(cx - frame_center_x) > horizontal_threshold:
            send_robot_command("left_move")
        else:
            send_robot_command("stopping")  # Ball is centrally located or too close

    # Display the frame
    cv2.imshow('White Ping Pong Balls Detection', frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and destroy all windows
cap.release()
cv2.destroyAllWindows()
