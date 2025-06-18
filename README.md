# Gesture-Controlled Robotic Hand

This project implements an assistive control system where a 3D-printed robotic hand is controlled in real-time by human hand gestures. The system uses a standard webcam, OpenCV for computer vision, and an Arduino microcontroller to actuate the hand's servo motors.


## Features
- **Real-Time Control:** Low-latency control of the robotic hand.
- **Gesture Recognition:** Detects the number of extended fingers (0 to 5) to trigger different poses.
- **Non-Invasive Interface:** Uses a standard webcam, requiring no physical sensors on the user.
- **Modular and Extensible:** The code is commented and structured for easy modification and addition of new gestures.

## Hardware Requirements
- Arduino UNO
- 5x MG90s (or similar) Servo Motors
- 3D Printed Robotic Hand/Palm
- Breadboard & Jumper Wires
- A standard USB Webcam
- A stable 5V power supply for the servos

## Software & Dependencies
- **Arduino IDE:** To upload the firmware to the Arduino board.
- **Python 3.7+**
- **Python Libraries:** Install them using pip:
  ```bash
  pip install opencv-python numpy pyserial
  ```

## Setup & Installation

### 1. Hardware Assembly
1.  Assemble your 3D-printed robotic hand with the five servo motors.
2.  Connect the **signal wires** of the servos to the Arduino's PWM pins:
    - Thumb Servo -> Pin 3
    - Index Finger Servo -> Pin 5
    - Middle Finger Servo -> Pin 6
    - Ring Finger Servo -> Pin 9
    - Pinky Finger Servo -> Pin 10
3.  Connect the **VCC (power)** and **GND (ground)** wires of all servos to an external 5V power supply. **Crucially, connect the Arduino's GND to the power supply's GND.** Do not power the servos directly from the Arduino's 5V pin.
4.  Connect the Arduino UNO to your computer via USB.

### 2. Arduino Firmware
1.  Open the `robotic_hand_arduino.ino` file in the Arduino IDE.
2.  Go to `Tools > Board` and select "Arduino UNO".
3.  Go to `Tools > Port` and select the COM port your Arduino is connected to. Note this port name (e.g., `COM3` on Windows, `/dev/ttyUSB0` on Linux).
4.  Click the "Upload" button.
5.  **Calibration:** Open the Arduino code and adjust the `_OPEN` and `_CLOSED` angle values for each servo to match your physical assembly perfectly.

### 3. Python Controller
1.  Open the `gesture_controller.py` file in a code editor.
2.  Find the line `ARDUINO_PORT = 'COM3'` and replace `'COM3'` with the port name you noted in the previous step.
3.  **HSV Calibration (Optional but Recommended):** The current skin detection values (`LOWER_SKIN` and `UPPER_SKIN`) might not work perfectly for your skin tone and lighting conditions. You may need to adjust them for better performance.

## How to Run
1.  Make sure the hardware is connected and the Arduino is running the firmware.
2.  Run the Python script from your terminal:
    ```bash
    python gesture_controller.py
    ```
3.  Two windows should appear: "Gesture Control" and potentially a "Mask" window if you uncomment it for debugging.
4.  Place your hand inside the green rectangle shown in the "Gesture Control" window.
5.  Show different numbers of fingers to control the robotic hand.
6.  Press the **'q'** key on your keyboard while the video window is active to quit the program.

## Project File Structure
```
.
├── gesture_controller.py      # Main Python script for gesture detection
├── robotic_hand_arduino.ino   # Firmware for the Arduino UNO
└── README.md                  # This setup and information guide
```
