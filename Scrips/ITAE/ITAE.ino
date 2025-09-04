#include <HCSR04.h> // Library for ultrasonic sensor

// Pins
byte triggerPin = 13;
byte echoPin = 12;
byte pwmPin = 6; // PWM pin for cooler

// PID variables
double setpoint = 50; // Setpoint in percentage
double input = 0;
double output = 0;

// PID parameters will be updated automatically
double Kp = 0.0;
double Ki = 0.0;
double Kd = 0.0;

// Anti-windup and filter parameters
double Tt = 3;       // Tracking time constant (seconds)
double integral = 0; // Integral term
double previous_error = 0; // Previous error for derivative calculation

// Derivative filter parameters
const double Tf = 0.02;     // Derivative filter time constant (seconds)
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

// Data collection for automatic tuning
const int maxDataPoints = 150; // Max number of data points to collect
double calibrationData[maxDataPoints];
unsigned long calibrationTime[maxDataPoints];
int dataIndex = 0;

// Process parameters calculated from calibration
double processK = 0.0;
double processTau = 0.0;
double processL = 0.0;

// Calibration times
const unsigned long calibZeroTime = 4000; // 4 seconds for zero calibration
const unsigned long calibMaxTime = 4000;  // 5 seconds for max calibration
const unsigned long postCalibTime = 4000; // 5 seconds for post-calibration

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

/**
 * @brief Applies the ITAE (Integral of Time-weighted Absolute Error) tuning method.
 *
 * This function calculates Kp, Ki, and Kd based on the process parameters
 * K (gain), L (dead time), and tau (time constant).
 *
 * @param K Process gain.
 * @param tau Process time constant.
 * @param L Process dead time.
 */
void ITAE(double K, double tau, double L) {
  if (K == 0 || L == 0 || tau == 0) { // Avoid division by zero and set default values
   // Serial.println("Error ITAE: No se puede sintonizar. Usando valores por defecto.");
    Kp = 0.6524;
    Ki = 0.584;
    Kd = 0.226;
    return;
  }
  
  double ratio = L / tau;
  
  // ITAE Formulas for PID Controller (setpoint tracking):
  double newKp = (0.586 / K) * pow(ratio, -0.916);
  double Ti = tau * (1.032 / 0.586) * pow(ratio, 0.738);
  double Td = tau * (0.381 / 0.586) * pow(ratio, 0.999);
  
  Kp = newKp;
  Ki = Kp / Ti;
  Kd = Kp * Td;
  
  Serial.println("PID sintonizado con ITAE.");
  Serial.print("Kp: "); Serial.println(Kp, 6);
  Serial.print("Ki: "); Serial.println(Ki, 6);
  Serial.print("Kd: "); Serial.println(Kd, 6);
}

/**
 * @brief Calculates the process parameters (K, tau, L) from the collected calibration data.
 */
void calculateProcessParameters() {
  if (dataIndex < 2) {
    Serial.println("Error: Datos insuficientes para el calculo.");
    return;
  }

  // 1. Calculate Process Gain (K)
  double outputChange = calibrationData[dataIndex - 1] - calibrationData[0];
  double inputChange = 180.0; // PWM step from 0 to 255
  processK = outputChange / inputChange;

  // 2. Calculate Dead Time (L)
  double initialValue = calibrationData[0];
  int firstChangeIndex = -1;
  // Look for a significant change (e.g., > 1% of the total change)
  for (int i = 1; i < dataIndex; i++) {
    if (abs(calibrationData[i] - initialValue) > 0.01 * abs(outputChange)) {
      firstChangeIndex = i;
      break;
    }
  }
  if (firstChangeIndex != -1) {
    processL = (calibrationTime[firstChangeIndex] - calibrationTime[0]) / 1000.0; // in seconds
  } else {
    processL = 0.0;
  }
  
  // 3. Calculate Time Constant (tau)
  double sixtyThreePercentPoint = initialValue + 0.632 * outputChange;
  int tauIndex = -1;
  // Find the point where the value is close to the 63.2% mark
  for (int i = firstChangeIndex; i < dataIndex; i++) {
    if (abs(calibrationData[i] - sixtyThreePercentPoint) < 0.5) { // within 0.5 cm
      tauIndex = i;
      break;
    }
  }
  if (tauIndex != -1) {
    processTau = (calibrationTime[tauIndex] - calibrationTime[firstChangeIndex]) / 1000.0; // in seconds
  } else {
    processTau = 0.0;
  }

 // Serial.println("\nParametros de proceso calculados:");
  //Serial.print("K: "); Serial.println(processK, 3);
  //Serial.print("tau: "); Serial.println(processTau, 3);
  //Serial.print("L: "); Serial.println(processL, 3);
}

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
  // Capture data points during the step response
  if (dataIndex < maxDataPoints) {
    calibrationData[dataIndex] = currentDistance;
    calibrationTime[dataIndex] = millis();
    dataIndex++;
  }

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
    calculateProcessParameters();
    ITAE(processK, processTau, processL);
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
    calculateProcessParameters();
    ITAE(processK, processTau, processL);
  // Calculate PID with anti-windup at fixed intervals
  if (millis() - lastPidTime >= 40) {
    computePidWithAntiWindup();
    lastPidTime = millis();
  }
  
  analogWrite(pwmPin, output);
  //Serial.println(input, 4); // Print current height percentage
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
  return constrain(percentage, 0.0, 100.00);
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
