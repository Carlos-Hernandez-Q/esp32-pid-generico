#include <Arduino.h>
#include <PIDControl.h>    // Librería propia que calcula el mando.
#include <ComandosPID.h>    // Librería propia para comandos y telemetría.
#include <math.h>
#include "Configuracion.h" // Ganancias, referencia, límites y periodo.
#include "Planta.h"        // Funciones que cada equipo completa para sus dispositivos.

namespace {
double Input = NAN;                          // Valor que entrega el sensor en unidades físicas.
double Setpoint = Config::REFERENCIA_INICIAL; // Objetivo en las mismas unidades que Input.
double Output = Config::SALIDA_SEGURA;        // Mando calculado para el actuador.

// El PID lee Input y Setpoint y actualiza el mismo Output que usa el actuador.
PIDControl miPID(Input, Output, Setpoint, Config::KP, Config::KI, Config::KD, Config::DIRECCION);

enum class Estado { Parado, Activo, Fallo };
Estado estado = Estado::Parado;
bool sistemaListo = false;
uint32_t ultimoMuestreo = 0;

bool leerEntrada();
bool aplicarParada(Estado destino);
bool arrancarPID();
bool detenerPID();
const char* nombreEstado();

ComandosPID comandos(Serial, miPID, Setpoint, Config::MEDICION_MIN, Config::MEDICION_MAX,
                     Config::PERIODO_TELEMETRIA_MS, arrancarPID, detenerPID);
}

void setup() {
    Serial.begin(Config::BAUDIOS); // Inicia el puerto para órdenes y telemetría.
    const bool limitesValidos = miPID.SetOutputLimits(Config::SALIDA_MIN, Config::SALIDA_MAX);
    const bool filtroValido = miPID.SetDerivativeFilter(Config::FILTRO_D_SEG);
    const bool hardwareListo = Planta::iniciar(); // Configura el sensor y el actuador de cada equipo.
    sistemaListo = limitesValidos && filtroValido && hardwareListo;
    if (!detenerPID()) Serial.println("ERROR inicializacion; revisar configuracion y Planta.cpp");
    ultimoMuestreo = millis();
    comandos.iniciar(); // Muestra las órdenes disponibles. El lazo espera START.
}

void loop() {
    if (sistemaListo && millis() - ultimoMuestreo >= Config::PERIODO_PID_MS) {
        const bool lecturaValida = leerEntrada(); // 1. El sensor actualiza Input en unidades físicas.
        const uint32_t instanteMedicion = millis();
        const double dtSegundos = (instanteMedicion - ultimoMuestreo) / 1000.0;
        ultimoMuestreo = instanteMedicion; // Marca cuándo quedó disponible esta lectura.

        if (estado == Estado::Activo) {
            if (!lecturaValida) {
                aplicarParada(Estado::Fallo);
                Serial.println("ERROR sensor; PID detenido");
            } else if (!miPID.Compute(dtSegundos)) { // 2. Calcula Output usando el intervalo real.
                aplicarParada(Estado::Fallo);
                Serial.println("ERROR calculo PID; PID detenido");
            } else if (!Planta::escribirActuador(Output)) { // 3. Entrega Output al actuador real.
                aplicarParada(Estado::Fallo);
                Serial.println("ERROR actuador; PID detenido");
            }
        }
    }

    comandos.actualizar(); // Procesa las órdenes seriales en la librería propia.
    comandos.publicarTelemetria(Input, Output, nombreEstado()); // Envía los datos del mismo lazo.
}

namespace {
// Funciones auxiliares para la lectura y los cambios de estado.
bool leerEntrada() {
    Input = NAN; // Una lectura inválida queda identificada como nan en el CSV.
    if (Planta::leerSensor(Input) && isfinite(Input) &&
        Input >= Config::MEDICION_MIN && Input <= Config::MEDICION_MAX) return true;
    Input = NAN;
    return false;
}

bool aplicarParada(Estado destino) {
    miPID.SetMode(ModoPID::Manual);   // Suspende el cálculo del PID.
    Output = Config::SALIDA_SEGURA;   // Registra el mando de parada solicitado.
    const bool aplicada = Planta::aplicarSalidaSegura();
    estado = aplicada ? destino : Estado::Fallo;
    if (!aplicada) Serial.println("ERROR salida segura no confirmada");
    return aplicada;
}

bool arrancarPID() {
    if (estado == Estado::Activo) return true;
    if (!sistemaListo || !leerEntrada()) {
        aplicarParada(Estado::Fallo);
        return false;
    }
    const uint32_t instanteMedicion = millis();
    if (!aplicarParada(Estado::Parado)) return false;
    if (!miPID.SetMode(ModoPID::Automatico)) { // Inicializa el controlador con Input y el mando seguro.
        estado = Estado::Fallo;
        Serial.println("ERROR parametros PID");
        return false;
    }
    estado = Estado::Activo;
    ultimoMuestreo = instanteMedicion;
    return true;
}

bool detenerPID() {
    return aplicarParada(sistemaListo ? Estado::Parado : Estado::Fallo) && sistemaListo;
}

const char* nombreEstado() {
    switch (estado) {
        case Estado::Parado: return "PARADO";
        case Estado::Activo: return "ACTIVO";
        default: return "FALLO";
    }
}

}
