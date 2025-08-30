#include <hcsr04.h>

#define TRIG_PIN 13
#define ECHO_PIN 12
#define PWM_PIN 6
#define STATUS_LED 13

HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, 0, 2509);

// Variables para el filtro promediativo
const int numLecturas = 10;
int lecturas[numLecturas];
int indice = 0;
long total = 0;

// Variables para calibración
int minDistancia = 0;
int maxDistancia = 2509;

// Variable para PWM manual
int pwmManual = 0;
bool modoManual = false;

// Variables para el timestamp
unsigned long startTime;
unsigned long currentTime;

// Estado del LED
bool ledState = false;
unsigned long lastBlink = 0;

// Buffer para datos seriales
String inputString = "";
bool stringComplete = false;

void setup() {
    Serial.begin(115200);
    
    // Esperar a que el puerto serial se conecte
    while (!Serial) {
        delay(10);
    }
    
    pinMode(PWM_PIN, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);
    analogWrite(PWM_PIN, 0);
    
    // Inicializar el array de lecturas
    for (int i = 0; i < numLecturas; i++) {
        lecturas[i] = 0;
    }
    
    startTime = millis();
    
    Serial.println("INIT: Sistema de levitación iniciado");
    Serial.println("FORMAT: timestamp,distancia,porcentaje,pwm");
}

void loop() {
    currentTime = millis() - startTime;
    
    // Parpadear LED para indicar actividad
    if (millis() - lastBlink > 500) {
        ledState = !ledState;
        digitalWrite(STATUS_LED, ledState);
        lastBlink = millis();
    }
    
    // Leer distancia cruda
    int distanciaCruda = hcsr04.distanceInMillimeters();
    
    // Aplicar filtro promediativo
    total = total - lecturas[indice];
    lecturas[indice] = distanciaCruda;
    total = total + lecturas[indice];
    indice = (indice + 1) % numLecturas;
    
    float distanciaFiltrada = total / (float)numLecturas;

    // Calcular porcentaje (0-100%)
    float porcentaje = 0;
    if (maxDistancia > minDistancia) {
        porcentaje = 100.0 * (distanciaFiltrada - minDistancia) / (maxDistancia - minDistancia);
        porcentaje = constrain(porcentaje, 0, 100);
    }

    // Manejo de comandos seriales
    if (stringComplete) {
        inputString.trim();
        
        if (inputString.length() > 0) {
            // Comando de PWM directo
            if (inputString.toInt() != 0 || inputString.equals("0")) {
                int valor = inputString.toInt();
                if (valor >= 0 && valor <= 255) {
                    pwmManual = valor;
                    modoManual = true;
                    analogWrite(PWM_PIN, pwmManual);
                }
            }
            // Comando con formato "pVALOR"
            else if (inputString.startsWith("p")) {
                int valor = inputString.substring(1).toInt();
                if (valor >= 0 && valor <= 255) {
                    pwmManual = valor;
                    modoManual = true;
                    analogWrite(PWM_PIN, pwmManual);
                }
            }
            else if (inputString.equals("auto")) {
                modoManual = false;
            }
            else if (inputString.equals("cal0")) {
                minDistancia = distanciaFiltrada;
            }
            else if (inputString.equals("cal100")) {
                maxDistancia = distanciaFiltrada;
            }
        }
        
        // Limpiar el buffer
        inputString = "";
        stringComplete = false;
    }

    // Control automático del PWM si no está en modo manual
    if (!modoManual) {
        if (distanciaFiltrada > 160) {
            analogWrite(PWM_PIN, 4);
        } else {
            analogWrite(PWM_PIN, 145);
        }
    }

    // Obtener el valor actual de PWM
    int pwmActual;
    if (modoManual) {
        pwmActual = pwmManual;
    } else {
        pwmActual = (distanciaFiltrada > 160) ? 4 : 145;
    }

    // Enviar datos en formato para Node-RED (una sola línea)
    Serial.print(currentTime);
    Serial.print(",");
    Serial.print(distanciaFiltrada, 2);  // 2 decimales
    Serial.print(",");
    Serial.print(porcentaje, 2);         // 2 decimales
    Serial.print(",");
    Serial.println(pwmActual);           // println agrega \n

    delay(100);  // Aumentar delay para evitar saturación
}

// Función que se ejecuta cuando llegan datos por serial
void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        
        if (inChar == '\n') {
            stringComplete = true;
        } else {
            inputString += inChar;
        }
    }
}