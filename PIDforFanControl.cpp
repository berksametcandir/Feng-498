#include <PID_v1.h> // Include the PID library

// Define pin numbers for distance sensor and motor fan
const int distanceSensorPin = A0;
const int fanSpeedPin = 9;

// Define variables for PID control
double Setpoint = 20; // Set the desired distance
double Input, Output;
double Kp = 2, Ki = 5, Kd = 1; // PID constants
bool isRunning = false; // Flag to indicate if the system is running
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);

  // Set fan speed pin as output
  pinMode(fanSpeedPin, OUTPUT);

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // Check if start or stop command received from ROS
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'start') {
      isRunning = true;
    } else if (command == 'stop') {
      isRunning = false;
      // Stop the fan
      analogWrite(fanSpeedPin, 0);
    }
  }

  // Read distance from sensor and adjust fan speed if system is running
  if (isRunning) {
    Input = analogRead(distanceSensorPin);
    // Compute PID
    myPID.Compute();

    // Adjust fan speed based on PID output
    int fanSpeed = map(Output, 0, 1023, 0, 255);
    analogWrite(fanSpeedPin, fanSpeed);

    // Print debug information
    Serial.print("Distance: ");
    Serial.print(Input);
    Serial.print(" | Fan Speed: ");
    Serial.println(fanSpeed);
  }

  delay(100); // Adjust delay as needed
}