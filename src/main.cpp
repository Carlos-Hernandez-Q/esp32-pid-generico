#include <Arduino.h>
#include <PIDControl.h>

namespace {
// Parámetros de la práctica LDR + LED.
constexpr uint8_t PIN_LDR = 34;      // Nodo del divisor de voltaje del LDR.
constexpr uint8_t PIN_LED = 18;      // LED externo con resistencia en serie.
constexpr double ENTRADA_MIN = 500; // Menor lectura del ADC que usa el PID.
constexpr double ENTRADA_MAX = 1000;
constexpr double SALIDA_MIN = 50;   // Menor mando permitido. El LED arranca aquí.
constexpr double SALIDA_MAX = 200;  // Mayor mando en PWM de 8 bits (0 a 255).
constexpr uint32_t PERIODO_MS = 50;

constexpr double KP = 0.20; // Corrección proporcional al error actual.
constexpr double KI = 0.30; // Corrige el error que se mantiene en el tiempo.
constexpr double KD = 0.0;  // La práctica empieza con control PI.
constexpr DireccionPID DIRECCION = DireccionPID::Directa; // Más brillo eleva la lectura.

double Input;             // Medición del sensor después de limitarla.
double Setpoint = 750.0;   // Objetivo en cuentas del ADC.
double Output = SALIDA_MIN;

PIDControl miPID(Input, Output, Setpoint, KP, KI, KD, DIRECCION);
bool listo = false;
uint32_t ultimaMuestra = 0;

// Conexión con el hardware. Estas funciones se adaptan al cambiar de equipo.
void iniciarHardware() {
    analogReadResolution(10); // La lectura del ESP32 queda entre 0 y 1023.
    analogSetPinAttenuation(PIN_LDR, ADC_11db);
    analogWriteResolution(8); // El mando PWM admite valores de 0 a 255.
}

double leerSensor() {
    return analogRead(PIN_LDR);
}

void escribirActuador(double mando) {
    analogWrite(PIN_LED, static_cast<int>(mando + 0.5)); // Redondea al entero más cercano.
}
}

void setup() {
    Serial.begin(115200);
    iniciarHardware();

    const double lectura = leerSensor();
    Input = constrain(lectura, ENTRADA_MIN, ENTRADA_MAX);
    listo = miPID.SetOutputLimits(SALIDA_MIN, SALIDA_MAX) &&
            miPID.SetClock([] { return static_cast<uint32_t>(millis()); }) &&
            miPID.SetMode(ModoPID::Automatico);
    if (listo) {
        Output = constrain(Output, SALIDA_MIN, SALIDA_MAX); // Aplica también los límites al arrancar.
        escribirActuador(Output);
    }
    ultimaMuestra = millis();
    Serial.println(listo ? "lectura_cruda,input,setpoint,output" : "ERROR al configurar el PID");
}

void loop() {
    const uint32_t ahora = millis();
    if (!listo || ahora - ultimaMuestra < PERIODO_MS) return;
    ultimaMuestra = ahora;

    const double lectura = leerSensor();                   // 1. Lee el sensor real.
    Input = constrain(lectura, ENTRADA_MIN, ENTRADA_MAX);  // 2. Limita la entrada a 500–1000.
    if (!miPID.Compute()) return;                          // 3. Calcula Output entre 50 y 200.
    escribirActuador(Output);                              // 4. Aplica el mando al LED.

    Serial.printf("%.2f,%.2f,%.2f,%.2f\n", lectura, Input, Setpoint, Output);
}
