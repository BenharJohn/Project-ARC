// robotic_hand_arduino.ino

#include <Servo.h>

// Create servo objects for 5 fingers
Servo thumbServo;
Servo indexServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

// IMPORTANT: Calibrate these angle values for your specific robotic hand assembly.
// The "open" and "closed" positions depend on how you've mounted the servos.
const int THUMB_OPEN = 10;
const int THUMB_CLOSED = 90;

const int INDEX_OPEN = 170;
const int INDEX_CLOSED = 80;

const int MIDDLE_OPEN = 170;
const int MIDDLE_CLOSED = 80;

const int RING_OPEN = 170;
const int RING_CLOSED = 80;

const int PINKY_OPEN = 170;
const int PINKY_CLOSED = 80;


void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);

  // Attach servos to their corresponding PWM pins
  thumbServo.attach(3);
  indexServo.attach(5);
  middleServo.attach(6);
  ringServo.attach(9);
  pinkyServo.attach(10);
  
  // Start with an open palm position
  openPalm();
  Serial.println("Arduino is ready. Robotic hand initialized.");
}

void loop() {
  // Check if there is data available to read from the serial port
  if (Serial.available() > 0) {
    // Read the incoming character command
    char command = Serial.read();

    // Execute action based on the command
    switch(command) {
      case '0': // Fist (0 fingers)
        closePalm();
        Serial.println("Received: 0 -> Closing Palm");
        break;
      case '1': // 1 finger
        pointIndex();
        Serial.println("Received: 1 -> Pointing Index");
        break;
      case '2': // 2 fingers
        showPeace();
        Serial.println("Received: 2 -> Showing Peace Sign");
        break;
      // You can add cases for 3 and 4 fingers if desired
      // case '3': ...
      // case '4': ...
      case '5': // 5 fingers
        openPalm();
        Serial.println("Received: 5 -> Opening Palm");
        break;
      default:
        // Optional: handle unknown commands
        Serial.println("Received unknown command.");
        break;
    }
  }
}

// ---- Helper Functions for Hand Gestures ----

// Function to open all fingers
void openPalm() {
  thumbServo.write(THUMB_OPEN);
  indexServo.write(INDEX_OPEN);
  middleServo.write(MIDDLE_OPEN);
  ringServo.write(RING_OPEN);
  pinkyServo.write(PINKY_OPEN);
}

// Function to close all fingers into a fist
void closePalm() {
  thumbServo.write(THUMB_CLOSED);
  indexServo.write(INDEX_CLOSED);
  middleServo.write(MIDDLE_CLOSED);
  ringServo.write(RING_CLOSED);
  pinkyServo.write(PINKY_CLOSED);
}

// Function to point with the index finger
void pointIndex() {
  thumbServo.write(THUMB_CLOSED);
  indexServo.write(INDEX_OPEN);
  middleServo.write(MIDDLE_CLOSED);
  ringServo.write(RING_CLOSED);
  pinkyServo.write(PINKY_CLOSED);
}

// Function for a "peace" sign (index and middle fingers open)
void showPeace() {
  thumbServo.write(THUMB_CLOSED);
  indexServo.write(INDEX_OPEN);
  middleServo.write(MIDDLE_OPEN);
  ringServo.write(RING_CLOSED);
  pinkyServo.write(PINKY_CLOSED);
}
