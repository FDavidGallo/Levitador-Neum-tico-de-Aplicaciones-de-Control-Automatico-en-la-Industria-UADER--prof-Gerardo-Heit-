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

void setup() {
    Serial.begin(9600);
    pinMode(PWM_PIN, OUTPUT);
    analogWrite(PWM_PIN, 0);
    
    // Inicializar el array de lecturas
    for (int i = 0; i < numLecturas; i++) {
        lecturas[i] = 0;
    }
}

void loop() {
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
        
        if (comando.startsWith("p")) {
            int valor = comando.substring(1).toInt();
            if (valor >= 0 && valor <= 255) {
                pwmManual = valor;
                modoManual = true;
                analogWrite(PWM_PIN, pwmManual);
                Serial.print("PWM manual: ");
                Serial.print(pwmManual);
                Serial.print(" (");
                Serial.print((pwmManual / 255.0) * 100);
                Serial.println("%)");
            }
        }
        else if (comando.equals("auto")) {
            modoManual = false;
            Serial.println("Modo automático activado");
        }
        else if (comando.startsWith("cal0")) {
            minDistancia = distanciaFiltrada;
            Serial.print("Nuevo 0% calibrado: ");
            Serial.println(minDistancia);
        }
        else if (comando.startsWith("cal100")) {
            maxDistancia = distanciaFiltrada;
            Serial.print("Nuevo 100% calibrado: ");
            Serial.println(maxDistancia);
        }
    }

    // Control automático del PWM si no está en modo manual
   // if (!modoManual) {
    //    if (distanciaFiltrada > 160) {
      //      analogWrite(PWM_PIN, 4);
       // } else {
         //   analogWrite(PWM_PIN, 145);
        //}
   // }

    // Enviar datos por serial
    Serial.print("Distancia: ");
    Serial.print(distanciaFiltrada);
    Serial.print("mm (");
    Serial.print(porcentaje);
    Serial.print("%) | PWM: ");
    Serial.print(modoManual ? pwmManual : (distanciaFiltrada > 160 ? 4 : 145));
    Serial.print(" (");
    Serial.print((modoManual ? pwmManual : (distanciaFiltrada > 160 ? 4 : 145)) / 255.0 * 100);
    Serial.println("%)");

    delay(20);
}