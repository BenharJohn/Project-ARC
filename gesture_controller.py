# gesture_controller.py

import cv2
import numpy as np
import serial
import time
import math

# --- SERIAL COMMUNICATION SETUP ---
# IMPORTANT: Replace 'COM3' with the port your Arduino is connected to.
# You can find the port name in the Arduino IDE under Tools > Port
ARDUINO_PORT = 'COM3' 
BAUD_RATE = 9600

try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # Wait for the serial connection to initialize
    print(f"Successfully connected to Arduino on {ARDUINO_PORT}")
except serial.SerialException as e:
    print(f"Error: Could not connect to Arduino on {ARDUINO_PORT}. {e}")
    print("Please check the port name and ensure the Arduino is connected.")
    exit()

# --- GESTURE DETECTION SETUP ---
# HSV color range for skin detection.
# IMPORTANT: You may need to calibrate these values for your skin tone and lighting.
# A useful tool for calibration: https://github.com/hari-prasath-me/Color-Picker-and-Color-Space-Converter
LOWER_SKIN = np.array([0, 48, 80], dtype=np.uint8)
UPPER_SKIN = np.array([20, 255, 255], dtype=np.uint8)

# --- VIDEO CAPTURE SETUP ---
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open video camera.")
    exit()

# Variables for tracking sent commands
last_sent_command = None
send_debounce_time = 1.0  # seconds
last_send_time = 0

print("\nStarting gesture detection. Place your hand in the green rectangle.")
print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Flip the frame horizontally for a more intuitive mirror-like display
    frame = cv2.flip(frame, 1)

    # Define the Region of Interest (ROI) for hand detection
    roi_x, roi_y, roi_w, roi_h = 300, 100, 300, 300
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h), (0, 255, 0), 2)
    roi = frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]

    # --- IMAGE PROCESSING ---
    # 1. Convert ROI to HSV color space
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 2. Create a mask based on the skin color range
    mask = cv2.inRange(hsv, LOWER_SKIN, UPPER_SKIN)

    # 3. Use morphological transformations to remove noise
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    mask = cv2.GaussianBlur(mask, (5, 5), 100) # Blurring the mask

    # --- CONTOUR AND GESTURE ANALYSIS ---
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    finger_count = 0
    
    try:
        # Find the contour with the largest area (assumed to be the hand)
        if len(contours) > 0:
            hand_contour = max(contours, key=lambda x: cv2.contourArea(x))
            
            # Check if the contour area is significant
            if cv2.contourArea(hand_contour) > 5000:
                cv2.drawContours(roi, [hand_contour], -1, (0, 0, 255), 3)

                # Find the convex hull of the hand contour
                hull = cv2.convexHull(hand_contour, returnPoints=False)
                # Find the convexity defects (the valleys between fingers)
                defects = cv2.convexityDefects(hand_contour, hull)

                if defects is not None:
                    finger_count = 1 # Start with one finger
                    for i in range(defects.shape[0]):
                        s, e, f, d = defects[i, 0]
                        start = tuple(hand_contour[s][0])
                        end = tuple(hand_contour[e][0])
                        far = tuple(hand_contour[f][0])
                        
                        # Calculate the angle of the defect using cosine rule
                        a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                        b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
                        c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
                        angle = math.acos((b**2 + c**2 - a**2) / (2 * b * c)) * (180 / math.pi)
                        
                        # If the angle is less than 90 degrees, it's a valid finger
                        if angle <= 90 and d > 20*256: # Check depth as well
                            finger_count += 1
                            cv2.circle(roi, far, 5, [255, 0, 0], -1)

    except Exception as e:
        print(f"Gesture analysis error: {e}")

    # Display the finger count
    if finger_count > 5: finger_count = 5
    cv2.putText(frame, f"Fingers: {finger_count}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # --- SEND COMMAND TO ARDUINO ---
    # Send command only if it's different from the last one and after a debounce delay
    command_to_send = str(finger_count)
    current_time = time.time()
    if command_to_send != last_sent_command and (current_time - last_send_time > send_debounce_time):
        try:
            arduino.write(command_to_send.encode())
            print(f"Sent command to Arduino: '{command_to_send}'")
            last_sent_command = command_to_send
            last_send_time = current_time
        except Exception as e:
            print(f"Failed to send data: {e}")

    # Display the frames
    cv2.imshow("Gesture Control", frame)
    # cv2.imshow("Mask", mask) # Uncomment to see the debug mask view

    # Check for 'q' key to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- CLEANUP ---
print("\nClosing application...")
# Send a final '0' to close the hand before exiting
if arduino.is_open:
    arduino.write(b'0')
    arduino.close()
cap.release()
cv2.destroyAllWindows()
