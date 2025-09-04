#include <HCSR04.h> // Library for ultrasonic sensor
#include <PID_v1.h> // PID library

// Pins
byte triggerPin = 13;
byte echoPin = 12;
byte pwmPin = 6; // PWM pin for cooler

// PID variables
double setpoint = 50; // Setpoint in percentage
double input = 0;
double output = 0;

// PID parameters based on Ziegler-Nichols tuning for the given transfer function
double Kp = 0.534;   // Proportional gain
double Ki = 0.0817;    // Integral gain
double Kd = 0.034;    // Derivative gain

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); // Create PID instance

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
const unsigned long calibZeroTime = 6000; // 6 seconds for zero calibration
const unsigned long calibMaxTime = 6000;  // 6 seconds for max calibration
const unsigned long postCalibTime = 3500; // 3.5 seconds for post-calibration

// System states
enum SystemState {
  CALIB_ZERO,
  CALIB_MAX,
  POST_CALIB,
  NORMAL_OPERATION
};
SystemState currentState = CALIB_ZERO;

unsigned long stateStartTime = 0;

void setup() {
  Serial.begin(115200);
  HCSR04.begin(triggerPin, echoPin);
  pinMode(pwmPin, OUTPUT);
  
  // Initialize filter
  for (int i = 0; i < filterSize; i++) {
    distanceBuffer[i] = 0.0;
  }
  
  // Configure PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(40); // Set sample time to 40 ms to match loop delay
  
  analogWrite(pwmPin, 0);
  stateStartTime = millis();
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
  }
}

void handleNormalOperation() {
  input = calculatePercentage();
  myPID.Compute();
  analogWrite(pwmPin, output);
  Serial.println(input, 2); // Print current height percentage
}

double calculatePercentage() {
  if (maxDistance <= minDistance) return 0.0;
  
  double percentage = 100.0 - 100.0 * (maxDistance - currentDistance) / (maxDistance - minDistance);
  return constrain(percentage, 0.0, 100.0);
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