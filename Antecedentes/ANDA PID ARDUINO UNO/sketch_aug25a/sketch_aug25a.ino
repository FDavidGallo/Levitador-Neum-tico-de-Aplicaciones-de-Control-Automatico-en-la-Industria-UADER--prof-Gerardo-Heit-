#include <HCSR04.h>

// Pines
byte triggerPin = 13;
byte echoPin = 12;
byte pwmPin = 6;

// Variables PID
double setpoint = 14;
double input, output;
double Kp = 4;    // Reducido para planta lenta
double Ki = 0.09;    // Reducido para planta lenta  
double Kd = 0.004;    // Reducido para planta lenta

double error, lastError = 100;
double integral = 0;
double derivative;

// Deadzone del PWM
const int PWM_MIN = 50;  // 52% de 255 = 132.6 ≈ 130
const int PWM_MAX = 255;
const int PWM_DEADZONE = PWM_MIN;

// Tiempo para el PID
unsigned long lastTime = 0;
unsigned long sampleTime = 55;  // Aumentado para planta lenta

// Filtro para mediciones
const double ALPHA = 0.8;  // Factor de filtrado (0-1)
double filteredInput = 0;

// Variables para Ziegler-Nichols
bool znTuning = false;
bool znStepResponse = false;
double znKu = 0;
double znPu = 0;
unsigned long znStartTime = 0;
unsigned long znLastPeakTime = 0;
double znLastValue = 0;
int znPeakCount = 0;

// Variables para entrada serial
String inputString = "";
bool waitingForInput = false;
int pwmValue = 0;

void setup() {
  Serial.begin(9600);
  HCSR04.begin(triggerPin, echoPin);
  pinMode(pwmPin, OUTPUT);
  
  Serial.println("Sistema de control PID para planta lenta");
  Serial.println("Setpoint: 15 cm");
  Serial.print("Deadzone PWM: ");
  Serial.print(PWM_DEADZONE);
  Serial.print(" (");
  Serial.print((PWM_DEADZONE * 100) / 255);
  Serial.println("%)");
  Serial.println("Comandos disponibles:");
  Serial.println("P - Cambiar setpoint");
  Serial.println("T - Ajustar parámetros PID manualmente");
  Serial.println("N - Sintonización automática Ziegler-Nichols");
  Serial.println("S - Mostrar configuración actual");
  Serial.println("C - Cancelar sintonización (si está activa)");
  Serial.println("------------------------------------------");
}

void loop() {
  handleSerial();
  
  if (!waitingForInput) {
    unsigned long now = millis();
    
    if (now - lastTime >= sampleTime) {
      lastTime = now;
      
      // Leer y filtrar distancia
      double* distances = HCSR04.measureDistanceCm();
      filteredInput = ALPHA * distances[0] + (1 - ALPHA) * filteredInput;
      input = filteredInput;
      
      if (znTuning) {
        handleZNTuning(now);
      } else {
        calculatePID();
        applyPWMWithDeadzone();
        printPIDInfo();
      }
    }
  }
  
  delay(50);  // Aumentado para planta lenta
}

void calculatePID() {
  error = setpoint - input;
  
  // Anti-windup: Limitar término integral
  integral += error;
  integral = constrain(integral, -1000, 1000);  // Límites más conservadores
  
  derivative = error - lastError;
  
  output = (Kp * error) + (Ki * integral * sampleTime/1000.0) + (Kd * derivative * 1000.0/sampleTime);
  
  lastError = error;
}

void applyPWMWithDeadzone() {
  // Aplicar deadzone y saturación
  if (output > 0) {
    pwmValue = map(output, 0, 255, PWM_DEADZONE, PWM_MAX);
  } else if (output < 0) {
    pwmValue = map(output, -255, 0, 0, PWM_DEADZONE);
  } else {
    pwmValue = PWM_DEADZONE;
  }
  
  pwmValue = constrain(pwmValue, 0, 255);
  analogWrite(pwmPin, pwmValue);
}

void handleZNTuning(unsigned long currentTime) {
  if (znStepResponse) {
    handleStepResponsePhase(currentTime);
  } else {
    findCriticalGain(currentTime);
  }
}

void findCriticalGain(unsigned long currentTime) {
  static double testKp = 1.0;
  static unsigned long lastKpIncrease = 0;
  
  if (currentTime - znStartTime > 30000) { // 30 segundos para planta lenta
    znTuning = false;
    Serial.println("Error: No se pudo encontrar ganancia crítica (planta muy lenta)");
    return;
  }
  
  // Aumentar Kp más lentamente para planta lenta
  if (currentTime - lastKpIncrease > 5000) { // Cada 5 segundos
    testKp += 1.0;
    lastKpIncrease = currentTime;
    integral = 0;
    lastError = 0;
    
    Serial.print("Probando Kp: ");
    Serial.println(testKp);
  }
  
  error = setpoint - input;
  output = testKp * error;
  applyPWMWithDeadzone();  // Usar deadzone durante sintonización
  
  // Detectar oscilación con mayor tolerancia para planta lenta
  if (currentTime - znStartTime > 10000) {
    detectOscillation(currentTime, input);
  }
  
  Serial.print("ZN Fase1 - Kp: ");
  Serial.print(testKp);
  Serial.print(" | Dist: ");
  Serial.print(input);
  Serial.print("cm | PWM: ");
  Serial.println(pwmValue);
}

void detectOscillation(unsigned long currentTime, double currentValue) {
  static bool rising = true;
  static double lastPeakValue = 0;
  
  if (currentValue > znLastValue && !rising) {
    rising = true;
    znPeakCount++;
    
    if (znPeakCount >= 2) {
      unsigned long period = currentTime - znLastPeakTime;
      znPu = period / 1000.0;
      znKu = Kp;
      
      Serial.println("\n¡Oscilación sostenida detectada!");
      Serial.print("Ganancia crítica Ku: ");
      Serial.println(znKu);
      Serial.print("Período crítico Pu: ");
      Serial.print(znPu);
      Serial.println(" segundos");
      
      znStepResponse = true;
      znStartTime = currentTime;
      znPeakCount = 0;
    }
    znLastPeakTime = currentTime;
    lastPeakValue = currentValue;
  }
  else if (currentValue < znLastValue && rising) {
    rising = false;
  }
  
  znLastValue = currentValue;
}

void handleStepResponsePhase(unsigned long currentTime) {
  // Para planta lenta, usar un escalón más prolongado
  static bool stepApplied = false;
  static unsigned long stepStartTime = 0;
  
  if (!stepApplied) {
    // Aplicar escalón del 40% (más conservador)
    analogWrite(pwmPin, map(128, 0, 255, PWM_DEADZONE, PWM_MAX));
    stepApplied = true;
    stepStartTime = currentTime;
    Serial.println("Aplicando escalón de prueba...");
  }
  
  Serial.print("ZN Fase2 - T: ");
  Serial.print((currentTime - stepStartTime) / 1000.0);
  Serial.print("s | Dist: ");
  Serial.println(input);
  
  // Monitorear por más tiempo para planta lenta
  if (currentTime - stepStartTime > 10000) { // 10 segundos
    finishZNTuning();
  }
}

void finishZNTuning() {
  // Fórmulas de Ziegler-Nichols modificadas para planta lenta
  Serial.println("\nAplicando Ziegler-Nichols modificado...");
  
  // Valores más conservadores para planta lenta
  Kp = 0.45 * znKu;       // Reducido de 0.6
  Ki = 1.2 * Kp / znPu;   // Reducido de 2.0
  Kd = Kp * znPu / 12.0;  // Reducido de 8.0
  
  integral = 0;
  lastError = 0;
  znTuning = false;
  znStepResponse = false;
  
  Serial.println("¡Sintonización completada!");
  Serial.print("Nuevos parámetros - Kp: ");
  Serial.print(Kp);
  Serial.print(", Ki: ");
  Serial.print(Ki);
  Serial.print(", Kd: ");
  Serial.println(Kd);
  Serial.println("Volviendo al control PID normal...");
}

void startZNTuning() {
  if (znTuning) {
    Serial.println("Ya en proceso de sintonización");
    return;
  }
  
  znTuning = true;
  znStepResponse = false;
  znStartTime = millis();
  znPeakCount = 0;
  znLastValue = 0;
  integral = 0;
  lastError = 0;
  filteredInput = setpoint;  // Inicializar filtro
  
  Serial.println("\n=== ZIEGLER-NICHOLS PARA PLANTA LENTA ===");
  Serial.println("ADVERTENCIA: Este proceso puede tomar 30-40 segundos");
  Serial.println("El sistema oscilará durante la sintonización");
  Serial.println("===========================================");
}

void handleSerial() {
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    
    if (waitingForInput) {
      if (incomingChar == '\n' || incomingChar == '\r') {
        processSerialInput();
      } else if (incomingChar == 27) { // ESC key
        waitingForInput = false;
        inputString = "";
        Serial.println("\nEntrada cancelada");
      } else {
        inputString += incomingChar;
      }
    } else {
      switch (toupper(incomingChar)) {
        case 'P': 
          startSetpointInput(); 
          break;
        case 'T': 
          startPIDTuning(); 
          break;
        case 'N': 
          startZNTuning(); 
          break;
        case 'S': 
          showCurrentSettings(); 
          break;
        case 'C': 
          if (znTuning) {
            znTuning = false;
            Serial.println("Sintonización cancelada");
          }
          break;
      }
    }
  }
}

void startSetpointInput() {
  waitingForInput = true;
  inputString = "";
  Serial.println("\nIngrese nueva distancia setpoint (cm) y presione Enter:");
  Serial.println("Recomendado: 10-30 cm para planta lenta");
}

void startPIDTuning() {
  waitingForInput = true;
  inputString = "";
  Serial.println("\nIngrese parámetros PID en formato Kp,Ki,Kd");
  Serial.println("Ejemplo: 20.0,0.5,10.0");
  Serial.println("Actual: " + String(Kp) + "," + String(Ki) + "," + String(Kd));
}

void processSerialInput() {
  if (inputString.length() > 0) {
    if (inputString.indexOf(',') != -1) {
      processPIDInput();
    } else {
      processSetpointInput();
    }
  }
  
  waitingForInput = false;
  inputString = "";
  Serial.println("Volviendo al control PID...");
}

void processSetpointInput() {
  double newSetpoint = inputString.toFloat();
  if (newSetpoint >= 8 && newSetpoint <= 40) {  // Rango más conservador
    setpoint = newSetpoint;
    integral = 0;
    lastError = 0;
    Serial.print("Setpoint cambiado a: ");
    Serial.print(setpoint);
    Serial.println(" cm");
  } else {
    Serial.println("Error: Setpoint debe estar entre 8 y 40 cm");
  }
}

void processPIDInput() {
  int firstComma = inputString.indexOf(',');
  int secondComma = inputString.indexOf(',', firstComma + 1);
  
  if (firstComma != -1 && secondComma != -1) {
    double newKp = inputString.substring(0, firstComma).toFloat();
    double newKi = inputString.substring(firstComma + 1, secondComma).toFloat();
    double newKd = inputString.substring(secondComma + 1).toFloat();
    
    if (newKp > 0 && newKi >= 0 && newKd >= 0) {
      Kp = newKp;
      Ki = newKi;
      Kd = newKd;
      integral = 0;
      lastError = 0;
      
      Serial.print("PID actualizado: Kp=");
      Serial.print(Kp);
      Serial.print(", Ki=");
      Serial.print(Ki);
      Serial.print(", Kd=");
      Serial.println(Kd);
    } else {
      Serial.println("Error: Valores PID deben ser positivos");
    }
  } else {
    Serial.println("Error: Formato incorrecto. Use: Kp,Ki,Kd");
  }
}

void printPIDInfo() {
  if (!znTuning) {
    Serial.print("Set: ");
    Serial.print(setpoint, 1);
    Serial.print("cm | Actual: ");
    Serial.print(input, 1);
    Serial.print("cm | Error: ");
    Serial.print(error, 1);
    Serial.print(" | PWM: ");
    Serial.print(pwmValue);
    Serial.print(" (");
    Serial.print((pwmValue * 100) / 255);
    Serial.println("%)");
  }
}

void showCurrentSettings() {
  Serial.println("\n--- Configuración Actual ---");
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.println(" cm");
  Serial.print("PID: Kp=");
  Serial.print(Kp);
  Serial.print(", Ki=");
  Serial.print(Ki);
  Serial.print(", Kd=");
  Serial.println(Kd);
  Serial.print("Deadzone PWM: ");
  Serial.print(PWM_DEADZONE);
  Serial.print(" (");
  Serial.print((PWM_DEADZONE * 100) / 255);
  Serial.println("%)");
  Serial.println("----------------------------");
}