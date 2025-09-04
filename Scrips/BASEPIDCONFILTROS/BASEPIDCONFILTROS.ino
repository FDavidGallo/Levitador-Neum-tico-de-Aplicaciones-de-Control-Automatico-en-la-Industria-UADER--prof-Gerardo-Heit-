#include <HCSR04.h> // Library for ultrasonic sensor

// Pins
byte triggerPin = 13;
byte echoPin = 12;
byte pwmPin = 6; // PWM pin for cooler

// PID variables
double setpoint = 50; // Setpoint in percentage
double input = 0;
double output = 0;

// PID parameters based on Ziegler-Nichols tuning for the given transfer function
double Kp = 1.65;   // Proportional gain
double Ki = 1.38;  // Integral gain
double Kd = 0.99;   // Derivative gain

// Anti-windup parameters
double Tt = 3;     // Tracking time constant (seconds)
double integral = 0; // Integral term
double previous_error = 0; // Previous error for derivative calculation

// Derivative filter parameters
const double Tf = 0.02;    // Derivative filter time constant (seconds)
double filtered_derivative = 0; // Filtered derivative term
double previous_filtered_derivative = 0; // Previous filtered derivative value

// Calibration variables
double minDistance = 0.0;
double maxDistance = 0.0;
double currentDistance = 0.0;

// Moving average filter
const int filterSize = 10;
double distanceBuffer[filterSize];
int bufferIndex = 0;
double distanceSum = 0.0;

// Calibration times
const unsigned long calibZeroTime = 4000; // 6 seconds for zero calibration
const unsigned long calibMaxTime = 5000;  // 6 seconds for max calibration
const unsigned long postCalibTime = 5000; // 3.5 seconds for post-calibration

// System states
enum SystemState {
  CALIB_ZERO,
  CALIB_MAX,
  POST_CALIB,
  NORMAL_OPERATION
};
SystemState currentState = CALIB_ZERO;

unsigned long stateStartTime = 0;
unsigned long lastPidTime = 0; // Time of last PID calculation
const double dt = 0.04;        // Sample time in seconds (40 ms)

void setup() {
  Serial.begin(115200);
  HCSR04.begin(triggerPin, echoPin);
  pinMode(pwmPin, OUTPUT);
  
  // Initialize filter
  for (int i = 0; i < filterSize; i++) {
    distanceBuffer[i] = 0.0;
  }
  
  analogWrite(pwmPin, 0);
  stateStartTime = millis();
  lastPidTime = millis();
}

void loop() {
  double* rawDistance = HCSR04.measureDistanceCm();
  if (rawDistance > 0) {
    applyMovingAverageFilter(rawDistance);
  }
  
  switch (currentState) {
    case CALIB_ZERO:
      handleCalibZeroState();
      break;
    case CALIB_MAX:
      handleCalibMaxState();
      break;
    case POST_CALIB:
      handlePostCalibState();
      break;
    case NORMAL_OPERATION:
      handleNormalOperation();
      break;
  }
  
  processSerialCommands();
  delay(40); // Main loop delay
}

void applyMovingAverageFilter(double* newValue) {
  distanceSum -= distanceBuffer[bufferIndex];
  distanceBuffer[bufferIndex] = *newValue;
  distanceSum += *newValue;
  currentDistance = distanceSum / filterSize;
  bufferIndex = (bufferIndex + 1) % filterSize;
}

void handleCalibZeroState() {
  analogWrite(pwmPin, 0);
  
  if (millis() - stateStartTime >= calibZeroTime) {
    minDistance = currentDistance;
    currentState = CALIB_MAX;
    stateStartTime = millis();
    analogWrite(pwmPin, 255);
  }
}

void handleCalibMaxState() {
  analogWrite(pwmPin, 255);
  
  if (millis() - stateStartTime >= calibMaxTime) {
    maxDistance = currentDistance;
    currentState = POST_CALIB;
    stateStartTime = millis();
    analogWrite(pwmPin, 0);
  }
}

void handlePostCalibState() {
  analogWrite(pwmPin, 0);
  
  if (millis() - stateStartTime >= postCalibTime) {
    currentState = NORMAL_OPERATION;
    // Reset PID terms when starting normal operation
    integral = 0;
    previous_error = 0;
    filtered_derivative = 0;
    previous_filtered_derivative = 0;
  }
}

void handleNormalOperation() {
  input = calculatePercentage();
  
  // Calculate PID with anti-windup at fixed intervals
  if (millis() - lastPidTime >= 40) {
    computePidWithAntiWindup();
    lastPidTime = millis();
  }
  
  analogWrite(pwmPin, output);
  Serial.println(input, 2); // Print current height percentage
}

void computePidWithAntiWindup() {
  // Calculate error
  double error = setpoint - input;
  
  // Calculate proportional term
  double proportional = Kp * error;
  
  // Calculate derivative term with filtering
  double derivative = (error - previous_error) / dt;
  
  // Apply first-order low-pass filter to derivative term
  double alpha = dt / (Tf + dt);
  filtered_derivative = (1 - alpha) * previous_filtered_derivative + alpha * derivative;
  
  // Scale by derivative gain
  double derivative_term = Kd * filtered_derivative;
  
  // Calculate unsaturated output
  double output_unsaturated = proportional + integral + derivative_term;
  
  // Apply output limits (saturation)
  output = constrain(output_unsaturated, 80, 180);
  
  // Back-calculation anti-windup
  // Calculate the difference between saturated and unsaturated output
  double saturation_difference = output - output_unsaturated;
  
  // Update integral term with anti-windup correction
  integral += Ki * error * dt + saturation_difference * (dt / Tt);
  
  // Store values for next iteration
  previous_error = error;
  previous_filtered_derivative = filtered_derivative;
}

double calculatePercentage() {
  if (maxDistance <= minDistance) return 0.0;
  
  double percentage = 100.0 - 100.0 * (maxDistance - currentDistance) / (maxDistance - minDistance);
  return constrain(percentage, 0.0, 100.100);
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (currentState == NORMAL_OPERATION) {
      // Set setpoint during normal operation
      int newSetpoint = input.toInt();
      if (newSetpoint >= 0 && newSetpoint <= 100) {
        setpoint = newSetpoint;
      }
    } else {
      // Manual PWM control during calibration
      int newPwm = input.toInt();
      if (newPwm >= 0 && newPwm <= 255) {
        analogWrite(pwmPin, newPwm);
      }
    }
  }
}