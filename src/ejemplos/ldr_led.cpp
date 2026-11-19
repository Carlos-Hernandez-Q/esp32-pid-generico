#include <Arduino.h>
#include <PIDControl.h>

namespace {
constexpr uint8_t PIN_LDR = 34;      // Nodo del divisor del LDR conectado a GPIO34.
constexpr uint8_t PIN_LED = 18;      // LED externo en GPIO18 con una resistencia en serie.
constexpr int ADC_MIN = 500;        // Menor lectura que se entrega al PID.
constexpr int ADC_MAX = 1000;       // Mayor lectura que se entrega al PID.
constexpr int PWM_MIN = 50;         // Menor mando del LED en una escala de 0 a 255.
constexpr int PWM_MAX = 200;        // Mayor mando del LED.
constexpr uint32_t PERIODO_MS = 50; // Se toma una lectura cada 50 ms aproximadamente.

constexpr double KP = 0.20; // Aumenta el PWM cuando la lectura está por debajo del objetivo.
constexpr double KI = 0.30; // Corrige un error que se mantiene en el tiempo.
constexpr double KD = 0.0;  // Para esta práctica se empieza con un PI.

double Input;             // Lectura real del LDR después de recortarla a 500–1000.
double Setpoint = 750.0;   // Objetivo de luz en cuentas del ADC, no en lux.
double Output = PWM_MIN;   // El mando arranca en 50 y el PID lo actualiza.

// Con el LDR hacia 3V3, iluminarlo aumenta Input: se usa dirección directa.
PIDControl miPID(Input, Output, Setpoint, KP, KI, KD, DireccionPID::Directa);
bool listo = false;
uint32_t ultimaMuestra = 0;
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(10);             // analogRead entrega de 0 a 1023.
    analogSetPinAttenuation(PIN_LDR, ADC_11db);
    analogWriteResolution(8);             // analogWrite admite de 0 a 255.
    analogWrite(PIN_LED, PWM_MIN);        // Empieza con el LED en el mando mínimo.

    const int lectura = analogRead(PIN_LDR); // Primera medición del sensor físico.
    Input = constrain(lectura, ADC_MIN, ADC_MAX);
    listo = miPID.SetOutputLimits(PWM_MIN, PWM_MAX) &&
            miPID.SetClock([] { return static_cast<uint32_t>(millis()); }) &&
            miPID.SetMode(ModoPID::Automatico); // La práctica empieza al encender.
    ultimaMuestra = millis();
    Serial.println(listo ? "adc_crudo,input,setpoint,pwm" : "ERROR al configurar el PID");
}

void loop() {
    const uint32_t ahora = millis();
    if (!listo || ahora - ultimaMuestra < PERIODO_MS) return;
    ultimaMuestra = ahora;

    const int lectura = analogRead(PIN_LDR);          // 1. Lee el LDR.
    Input = constrain(lectura, ADC_MIN, ADC_MAX);      // 2. Recorta a 500–1000.
    if (!miPID.Compute()) return;                      // 3. Calcula Output dentro de 50–200.
    const int pwm = static_cast<int>(Output + 0.5);    // Redondea el mando a un entero.
    analogWrite(PIN_LED, pwm);                         // 4. Manda el PWM al LED.

    Serial.printf("%d,%d,%.0f,%d\n", lectura, static_cast<int>(Input), Setpoint, pwm);
}
