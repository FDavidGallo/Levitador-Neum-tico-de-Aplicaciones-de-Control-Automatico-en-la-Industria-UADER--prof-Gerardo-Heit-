#include <hcsr04.h>

#define TRIG_PIN 13
#define ECHO_PIN 12
#define PWM_PIN 6

HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, 0, 2509);

// Variables para el filtro promediativo
const int numLecturas = 10;
int lecturas[numLecturas];
int indice = 0;
long total = 0;

// Variables para calibración
int minDistancia = 0;    // 0%
int maxDistancia = 2509; // 100% (máximo del sensor)

// Variable para PWM manual
int pwmManual = 0;
bool modoManual = false;

// Variables para el timestamp
unsigned long startTime;
unsigned long currentTime;

// Función para verificar si una cadena es numérica
bool esNumerico(String str) {
    for (byte i = 0; i < str.length(); i++) {
        if (!isDigit(str[i])) {
            return false;
        }
    }
    return true;
}

void setup() {
    Serial.begin(115200); // Cambiado a 115200 baudios para Node-RED
    pinMode(PWM_PIN, OUTPUT);
    analogWrite(PWM_PIN, 0);
    
    // Inicializar el array de lecturas
    for (int i = 0; i < numLecturas; i++) {
        lecturas[i] = 0;
    }
    
    startTime = millis(); // Iniciar el contador de tiempo
}

void loop() {
    currentTime = millis() - startTime; // Tiempo transcurrido en milisegundos
    
    // Leer distancia cruda
    int distanciaCruda = hcsr04.distanceInMillimeters();
    
    // Aplicar filtro promediativo
    total = total - lecturas[indice];
    lecturas[indice] = distanciaCruda;
    total = total + lecturas[indice];
    indice = (indice + 1) % numLecturas;
    
    float distanciaFiltrada = total / (float)numLecturas;

    // Calcular porcentaje (0-100%)
    float porcentaje = map(distanciaFiltrada, minDistancia, maxDistancia, 0, 100.0);
    porcentaje = constrain(porcentaje, 0, 100);

    // Manejo de comandos seriales
    if (Serial.available() > 0) {
        String comando = Serial.readStringUntil('\n');
        comando.trim();
        
        int valorPWM = -1;
        
        // Detectar si es un comando de PWM de Node-RED (solo número)
        if (esNumerico(comando)) {
            valorPWM = comando.toInt();
        }
        // Detectar si es un comando manual con formato "pVALOR"
        else if (comando.startsWith("p")) {
            valorPWM = comando.substring(1).toInt();
        }
        
        // Aplicar el valor de PWM si es válido
        if (valorPWM >= 0 && valorPWM <= 255) {
            pwmManual = valorPWM;
            modoManual = true;
            analogWrite(PWM_PIN, pwmManual);
        }
        else if (comando.equals("auto")) {
            modoManual = false;
        }
        else if (comando.startsWith("cal0")) {
            minDistancia = distanciaFiltrada;
        }
        else if (comando.startsWith("cal100")) {
            maxDistancia = distanciaFiltrada;
        }
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

    // Enviar datos en formato para Node-RED: timestamp,distancia,pwm
    Serial.print(currentTime);
    Serial.print(",");
    Serial.print(distanciaFiltrada);
    Serial.print(",");
    Serial.println(pwmActual);

    delay(20);
}