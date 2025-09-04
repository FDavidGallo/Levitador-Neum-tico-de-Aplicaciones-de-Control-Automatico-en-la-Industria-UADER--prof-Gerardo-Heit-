#include <HCSR04.h>

// Pines
byte triggerPin = 13;
byte echoPin = 12;
byte pwmPin = 6;

// Variables para calibración
double minDistance = 3.0;
double maxDistance = 0.0;
double currentDistance = 0.0;
int pwmValue = 0;

// Filtro promedio móvil
const int filterSize = 5;
double distanceBuffer[filterSize];
int bufferIndex = 0;
double distanceSum = 0.0;

// Tiempos de calibración
const unsigned long calibZeroTime = 4000;  // 50 segundos
const unsigned long calibMaxTime = 4000;   // 60 segundos
const unsigned long postCalibTime = 8000;  // 60 segundos después de calibración

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

  // Inicializar filtro promedio móvil
  for (int i = 0; i < filterSize; i++) {
    distanceBuffer[i] = 0.0;
  }

  analogWrite(pwmPin, 0);
  stateStartTime = millis();

  // Serial.println("Sistema iniciado. Iniciando calibración...");
}

void loop() {
  // Leer distancia actual con filtro
  double* rawDistance = HCSR04.measureDistanceCm();
  if (rawDistance > 0) {  // Solo procesar lecturas válidas
    applyMovingAverageFilter(rawDistance);
  }

  // Máquina de estados para el proceso de calibración
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

  // Procesar comandos seriales si están disponibles
  processSerialCommands();

  delay(40);  // Pequeña pausa para estabilidad
}

void applyMovingAverageFilter(double* newValue) {
  // Restar el valor más antiguo de la suma
  distanceSum -= distanceBuffer[bufferIndex];

  // Agregar el nuevo valor al buffer y a la suma
  distanceBuffer[bufferIndex] = *newValue;
  distanceSum += *newValue;

  // Calcular el promedio
  currentDistance = distanceSum / filterSize;

  // Avanzar el índice circular
  bufferIndex = (bufferIndex + 1) % filterSize;
}

void handleCalibZeroState() {
  analogWrite(pwmPin, 0);

  if (millis() - stateStartTime >= calibZeroTime) {
    minDistance = currentDistance;
    // Serial.print("Calibración 0% completada. Distancia mínima: ");
    // Serial.print(minDistance, 2);
    //Serial.println(" cm");

    // Cambiar a estado de calibración máxima
    currentState = CALIB_MAX;
    stateStartTime = millis();
    analogWrite(pwmPin, 255);
  }
}

void handleCalibMaxState() {
  analogWrite(pwmPin, 255);

  if (millis() - stateStartTime >= calibMaxTime) {
    maxDistance = currentDistance;
    //Serial.print("Calibración 100% completada. Distancia máxima: ");
    // Serial.print(maxDistance, 2);
    //Serial.println(" cm");

    // Cambiar a estado post-calibración
    currentState = POST_CALIB;
    stateStartTime = millis();
    analogWrite(pwmPin, 0);
  }
}

void handlePostCalibState() {
  analogWrite(pwmPin, 0);

  if (millis() - stateStartTime >= postCalibTime) {
    //Serial.println("Calibración completa. Sistema en operación normal.");
    currentState = NORMAL_OPERATION;
  }
}

void handleNormalOperation() {
  // Calcular porcentaje basado en la calibración
  double percentage = calculatePercentage();
  if (percentage <= 4.5) {
    percentage = 0.0;
  }
  // Enviar porcentaje por serial con 2 decimales
  Serial.println(percentage, 2);
}

double calculatePercentage() {
  if (maxDistance <= minDistance) {
    return 0.0;  // Evitar división por cero
  }

  // Calcular porcentaje (0-100%) basado en la distancia actual
  double percentage = 100.0 - 100.0 * (maxDistance - currentDistance) / (maxDistance - minDistance);

  // Limitar el porcentaje entre 0 y 100
  percentage = constrain(percentage, 0.0, 100.0);

  return percentage;
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int newPwm = input.toInt();

    // Validar que el valor esté en el rango correcto
    if (newPwm >= 0 && newPwm <= 255) {
      pwmValue = newPwm;
      analogWrite(pwmPin, pwmValue);
    }
  }
}
