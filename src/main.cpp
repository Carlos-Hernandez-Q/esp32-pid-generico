#include "Configuracion.h" // Aquí se editan ganancias, referencia, periodo y límites.
#include "LazoPID.h"       // Coordina la lectura del sensor, la salida y el serial.
#include <PIDControl.h>   // Calcula Output a partir de Input y Setpoint.
#include <math.h>         // NAN indica que aún no hay una medición válida.

namespace {
double Input = NAN;                          // Medición del sensor.
double Setpoint = Config::REFERENCIA_INICIAL; // Objetivo en las mismas unidades que Input.
double Output = Config::SALIDA_SEGURA;        // Mando para el actuador.

PIDControl miPID(Input, Output, Setpoint, Config::KP, Config::KI, Config::KD, Config::DIRECCION);
LazoPID lazo(miPID, Input, Output, Setpoint);
}

void setup() {
    const bool limites = miPID.SetOutputLimits(Config::SALIDA_MIN, Config::SALIDA_MAX);
    const bool filtro = miPID.SetDerivativeFilter(Config::FILTRO_D_SEG);
    lazo.iniciar(limites && filtro); // Inicia hardware y serial. El lazo espera START.
}

void loop() {
    if (lazo.leerSensor()) {                         // 1. La nueva medición queda en Input.
        const bool calculoValido = miPID.Compute();   // 2. El PID calcula Output usando su reloj.
        lazo.aplicarSalida(Output, calculoValido);    // 3. Entrega el mando válido al actuador.
    }
    lazo.atenderSerial(); // Procesa órdenes y publica las mediciones.
}
