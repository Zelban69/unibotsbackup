from inference import get_roboflow_model
import supervision as sv
import cv2
import serial
import pandas as pd
import time  
import numpy as np

# --- Initializations ---

# Initialize the webcam
camera = cv2.VideoCapture(0)
# Set resolution of camera
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# Check if the resolution was set correctly
ret, frame = camera.read()
if ret:
    # Print the resolution of the frame
    height, width, channels = frame.shape
    print(f"Set Resolution: Width = 640, Height = 480")
    print(f"Actual Resolution: Width = {width}, Height = {height}")
else:
    print("Failed to grab frame from camera.")
    exit()  # Exit if camera fails

# Initialize serial communication with Arduino
try:
    ser = serial.Serial(port="COM3", baudrate=9600, timeout=0.1)
    print("Serial connection established.")
except serial.SerialException as e:
    print(f"Error opening serial port COM3: {e}")
    exit()


# Load a pre-trained model
# *** CHECK THIS: Ensure this model ID is correct ***
model = get_roboflow_model(model_id="ping-pong-finder-w6mxk/9")

# Create supervision annotators
bounding_box_annotator = sv.BoundingBoxAnnotator()
label_annotator = sv.LabelAnnotator()


# --- Command Logic Variables ---
command_counts = {
    "right_move": 0,
    "left_move": 0,
    "forward_move": 0,
    "stop": 0,
    "searching": 0,
}
last_sent_command = None  # Keep track of last sent command
COMMAND_THRESHOLD = 3  # Number of frames command must persist (Tune if needed)
SEARCH_THRESHOLD = 5  # Separate threshold for searching (Tune if needed)


# --- Main Loop ---
while True:
    # Capture frame-by-frame
    ret, frame = camera.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Optional: Adjust contrast/brightness
    frame_adjusted = cv2.convertScaleAbs(frame, alpha=0.9, beta=0)

    # Run inference on the current frame
    results = model.infer(frame_adjusted)  # Use adjusted frame for inference

    image_area = frame.shape[0] * frame.shape[1]
    # Load the results into the supervision Detections API
    detections = sv.Detections.from_inference(
        results[0].dict(by_alias=True, exclude_none=True)
    )
    # Confidence level filtering
    detections = detections[detections.confidence > 0.6]
    # Class ID filtering
    # *** CHECK THIS: Ensure class_id == 2 is correct for your ping pong ball ***
    detections = detections[detections.class_id == 2]

    # Define thresholds for movement based on frame dimensions
    # *** TUNE THIS BASED ON DEBUG OUTPUT BELOW ***
    # You mentioned changing this to / 6 in the code you pasted, keeping it here
    horizontal_threshold = frame.shape[1] / 9
    frame_center_x = frame.shape[1] / 2

    current_command_intent = None  # What command does the current frame suggest?
    area_percent = 0  # Initialize area_percent

    # --- Detection Logic ---
    if len(detections) > 0:
        # Find the largest detection
        areas_pixels = detections.box_area
        max_area_index = np.argmax(areas_pixels)

        # Select the single largest detection
        largest_detection = detections[[max_area_index]]

        # Calculate area percentage
        area_percent = (largest_detection.box_area[0] / image_area) * 100

        # Get center coordinates of the largest detection
        x1, y1, x2, y2 = largest_detection.xyxy[0]
        cx = (x1 + x2) / 2

        # Calculate deviation
        horizontal_deviation = cx - frame_center_x

        # --- START: Debugging Print Statements ---
        print(f"--- Frame Debug ---")
        print(f"cx: {cx:.2f}, center_x: {frame_center_x:.2f}")
        print(
            f"Horiz Dev: {horizontal_deviation:.2f}, Threshold: {horizontal_threshold:.2f}"
        )
        print(f"Area %: {area_percent:.2f}")
        # Evaluate conditions for clarity in printout
        # *** TUNE THIS AREA THRESHOLD IF NEEDED ***
        is_too_close = area_percent > 2.5
        is_off_center = abs(horizontal_deviation) > horizontal_threshold
        print(f"Is Too Close (Area > 2.5): {is_too_close}")
        print(f"Is Off Center (Abs Dev > Threshold): {is_off_center}")
        # --- END: Debugging Print Statements ---

        # --- START: Modified Decision Logic ---
        if is_too_close: # Stop if too close
            current_command_intent = "stop"
        elif is_off_center: # Turn if off-center
            if horizontal_deviation > 0:  # Ball is to the right
                current_command_intent = "right_move" # Command to turn right
            else:  # Ball is to the left
                current_command_intent = "left_move" # Command to turn left
        else: # If NOT too close AND NOT off-center, then move forward
            current_command_intent = "forward_move"
        # --- END: Modified Decision Logic ---

        # --- Print resulting intent and state ---
        print(f"Intent: {current_command_intent}")
        print(f"Counts: {command_counts}")
        print(f"Last Sent: {last_sent_command}")
        print(f"--------------------")


    else:  # No detections
        current_command_intent = "searching"
        # --- Debugging Print Statements for No Detection ---
        print("--- Frame Debug ---")
        print("No detections")
        print(f"Intent: {current_command_intent}")
        print(f"Counts: {command_counts}")
        print(f"Last Sent: {last_sent_command}")
        print(f"--------------------")


    # --- Command Sending Logic ---
    if current_command_intent:
        command_counts[current_command_intent] += 1

        # Determine threshold based on command type
        threshold = (
            SEARCH_THRESHOLD
            if current_command_intent == "searching"
            else COMMAND_THRESHOLD
        )

        # Check if threshold is met for the intended command
        if command_counts[current_command_intent] >= threshold:
            # Prevent sending the same command repeatedly, except for the 'stop' sequence
            if (
                current_command_intent != last_sent_command
                or current_command_intent == "stop"
            ):

                # *** Special handling for 'stop' command ***
                # This block executes the sequence you requested:
                # stop -> forward -> wait 1.5s -> stop
                if current_command_intent == "stop":
                    print(f"Stop condition met (Area: {area_percent:.2f}%)")
                    # 1. Send the initial stop command
                    ser.write(b"stop\n")
                    print("SENT: stop (initial)")

                    # 2. Send the forward command for final approach
                    ser.write(b"forward_move\n")
                    print("SENT: forward_move (final approach)")

                    # 3. Pause during approach (Tune this time if needed)
                    time.sleep(1.5)

                    # 4. Send the final stop command
                    ser.write(b"stop\n")
                    print("SENT: stop (final)")

                    # Record that the last action sequence was the stop sequence
                    last_sent_command = "stop"

                # *** Handling for other commands ***
                else:
                    command_bytes = f"{current_command_intent}\n".encode(
                        "utf-8"
                    )
                    ser.write(command_bytes)
                    print(f"SENT: {current_command_intent}")
                    last_sent_command = current_command_intent

            # Reset the count for the command that just met the threshold
            command_counts[current_command_intent] = 0
            # Reset other counts to ensure only one command builds up
            for cmd in command_counts:
                if cmd != current_command_intent:
                    command_counts[cmd] = 0

    # --- Annotation and Display (Standard) ---
    display_detections = (
        detections[[max_area_index]]
        if len(detections) > 0
        else sv.Detections.empty()
    )

    annotated_frame = bounding_box_annotator.annotate(
        scene=frame.copy(), detections=display_detections
    )
    annotated_frame = label_annotator.annotate(
        scene=annotated_frame, detections=display_detections
    )

    # Display the annotated frame
    cv2.imshow("Frame", annotated_frame)

    # --- Quit Condition (Standard, includes final stop) ---
    if cv2.waitKey(1) & 0xFF == ord("q"):
        print("Quitting...")
        try:
            # Send a final stop command before exiting
            ser.write(b"stop\n")
            print("SENT: stop (on quit)")
        except serial.SerialException as e:
            print(f"Serial write error on quit: {e}")
        break

# --- Cleanup (Standard) ---
if ser.is_open:
    ser.close()
    print("Serial port closed.")
camera.release()
cv2.destroyAllWindows()
print("Camera released and windows closed.")

