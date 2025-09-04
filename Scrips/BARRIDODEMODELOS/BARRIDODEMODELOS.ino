#include <HCSR04.h>

// Pins
byte triggerPin = 13;
byte echoPin = 12;
byte pwmPin = 6;

// PID variables
double setpoint = 50;
double input = 0;
double output = 0;

// PID gains (added back)
double Kp = 0.45;   // Proportional gain
double Ki = 0.5;    // Integral gain
double Kd = 0.7;    // Derivative gain

// Anti-windup parameters
double Tt = 3;
double integral = 0;
double previous_error = 0;

// Derivative filter parameters
const double Tf = 0.02;
double filtered_derivative = 0;
double previous_filtered_derivative = 0;

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
const unsigned long calibZeroTime = 4000;
const unsigned long calibMaxTime = 5000;
const unsigned long postCalibTime = 5000;

// Phase configuration
struct Phase {
  int serialCode;
  double Kp;
  double Ki;
  double Kd;
};

Phase phases[] = {
  {180, 0.45, 0.5, 0.7},
  {200, 1.599, 0.555, 0.307},
  {220, 3.261, 0.783, 0.463},
  {240, 4.65, 1.27, 2.255},
  {250, 1.38, 0.56, 0.481}
};
const int numPhases = 5;

// System states
enum SystemState {
  CALIB_ZERO,
  CALIB_MAX,
  POST_CALIB,
  NORMAL_OPERATION,
  PHASE_PAUSE
};
SystemState currentState = CALIB_ZERO;

// Timing variables
unsigned long stateStartTime = 0;
unsigned long lastPidTime = 0;
unsigned long phaseStartTime = 0;
const double dt = 0.04;
const unsigned long phaseDuration = 145000;  // 3 minutes
const unsigned long pauseDuration = 5000;    // 5 seconds

// Phase control
int currentPhase = 0;

void setup() {
  Serial.begin(115200);
  HCSR04.begin(triggerPin, echoPin);
  pinMode(pwmPin, OUTPUT);
  
  for (int i = 0; i < filterSize; i++) {
    distanceBuffer[i] = 0.0;
  }
  
  analogWrite(pwmPin, 0);
  stateStartTime = millis();
  lastPidTime = millis();
}

void loop() {
  double* rawDistance = HCSR04.measureDistanceCm();
  if (rawDistance != nullptr && *rawDistance > 0) {
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
    case PHASE_PAUSE:
      handlePhasePause();
      break;
  }
  
  processSerialCommands();
  delay(40);
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
    startNewPhase(0);
    currentState = NORMAL_OPERATION;
  }
}

void handleNormalOperation() {
  input = calculatePercentage();
  
  if (millis() - lastPidTime >= 40) {
    computePidWithAntiWindup();
    lastPidTime = millis();
  }
  
  analogWrite(pwmPin, output);
  Serial.println(input, 2);

  // Check if phase time completed
  if (millis() - phaseStartTime >= phaseDuration) {
    currentState = PHASE_PAUSE;
    stateStartTime = millis();
    analogWrite(pwmPin, 0);
  }
}

void handlePhasePause() {
  analogWrite(pwmPin, 0);
  
  if (millis() - stateStartTime >= pauseDuration) {
    currentPhase = (currentPhase + 1) % numPhases;
    startNewPhase(currentPhase);
    currentState = NORMAL_OPERATION;
  }
}

void startNewPhase(int phaseIndex) {
  Phase p = phases[phaseIndex];
  Serial.println(p.serialCode);
  
  // Update PID gains
  Kp = p.Kp;
  Ki = p.Ki;
  Kd = p.Kd;
  
  // Reset PID terms
  integral = 0;
  previous_error = 0;
  filtered_derivative = 0;
  previous_filtered_derivative = 0;
  
  phaseStartTime = millis();
}

void computePidWithAntiWindup() {
  double error = setpoint - input;
  double proportional = Kp * error;
  
  double derivative = (error - previous_error) / dt;
  double alpha = dt / (Tf + dt);
  filtered_derivative = (1 - alpha) * previous_filtered_derivative + alpha * derivative;
  double derivative_term = Kd * filtered_derivative;
  
  double output_unsaturated = proportional + integral + derivative_term;
  output = constrain(output_unsaturated, 80, 180);
  
  double saturation_difference = output - output_unsaturated;
  integral += Ki * error * dt + saturation_difference * (dt / Tt);
  
  previous_error = error;
  previous_filtered_derivative = filtered_derivative;
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
      int newSetpoint = input.toInt();
      if (newSetpoint >= 0 && newSetpoint <= 100) {
        setpoint = newSetpoint;
      }
    } else if (currentState != PHASE_PAUSE) {
      int newPwm = input.toInt();
      if (newPwm >= 0 && newPwm <= 255) {
        analogWrite(pwmPin, newPwm);
      }
    }
  }
}