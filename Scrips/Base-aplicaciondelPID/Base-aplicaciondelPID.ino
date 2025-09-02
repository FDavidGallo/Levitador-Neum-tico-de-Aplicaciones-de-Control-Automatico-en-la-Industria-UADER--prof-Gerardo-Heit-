#include <HCSR04.h> // Llamamos a la función para medir la altura
#include <PID_v1.h> // Llamamos a la función para implementar el PID

// Pines
byte triggerPin = 13;
byte echoPin = 12;
byte pwmPin = 6; // De aca va para el cooler

// Variables PID
double setpoint = 50; //
double input = 0;
double output = 0;

// Parámetros PID (ajustar según necesidades)
double Kp = 0.21;
double Ki = 5.0;
double Kd = 1.0;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); // Instanciamos el objeto PID

// Variables para calibración
double minDistance = 0.0;
double maxDistance = 0.0;
double currentDistance = 0.0;

// Filtro promedio móvil
const int filterSize = 10;
double distanceBuffer[filterSize];
int bufferIndex = 0;
double distanceSum = 0.0;

// Tiempos de calibración
const unsigned long calibZeroTime = 6000; //Peramo' 6 segundos por calibracion
const unsigned long calibMaxTime = 6000;
const unsigned long postCalibTime = 3500; // Pa que la pelota quede en 0 al iniciar

// Estados del sistema
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
  
  // Inicializar filtro
  for (int i = 0; i < filterSize; i++) {
    distanceBuffer[i] = 0.0;
  }
  
  // Configurar PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  
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
    currentState = NORMAL_OPERATION;
  }
}

void handleNormalOperation() {
  input = calculatePercentage();
  myPID.Compute();
  analogWrite(pwmPin, output);
  Serial.println(input, 2);
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
      // En estado normal, recibir setpoint (0-100)
      int newSetpoint = input.toInt();
      if (newSetpoint >= 0 && newSetpoint <= 100) {
        setpoint = newSetpoint;
      }
    } else {
      // Durante calibración, control manual del PWM (0-255)
      int newPwm = input.toInt();
      if (newPwm >= 0 && newPwm <= 255) {
        analogWrite(pwmPin, newPwm);
      }
    }
  }
}