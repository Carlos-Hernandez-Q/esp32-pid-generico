#include "LazoPID.h"
#include "Configuracion.h"
#include "Planta.h"
#include <math.h>

LazoPID::LazoPID(PIDControl& pid, double& input, double& output, double& setpoint)
    : pid_(pid), input_(input), output_(output),
      comandos_(Serial, pid, setpoint, Config::MEDICION_MIN, Config::MEDICION_MAX,
                Config::PERIODO_TELEMETRIA_MS, [this] { return arrancar(); },
                [this] { return detener(); }) {}

void LazoPID::iniciar(bool configuracionValida) {
    Serial.begin(Config::BAUDIOS);
    const bool relojListo = pid_.SetClock([] { return static_cast<uint32_t>(millis()); });
    const bool hardwareListo = Planta::iniciar();
    sistemaListo_ = configuracionValida && relojListo && hardwareListo;
    if (!detener()) Serial.println("ERROR inicializacion; revisar configuracion y Planta.cpp");
    ultimoMuestreo_ = millis();
    comandos_.iniciar();
}

bool LazoPID::leerSensor() {
    if (!sistemaListo_ || millis() - ultimoMuestreo_ < Config::PERIODO_PID_MS) return false;

    const bool lecturaValida = leerEntrada();
    ultimoMuestreo_ = millis();
    if (estado_ != Estado::Activo) return false;
    if (!lecturaValida) {
        aplicarParada(Estado::Fallo);
        Serial.println("ERROR sensor; PID detenido");
    }
    return lecturaValida;
}

void LazoPID::aplicarSalida(double output, bool calculoValido) {
    if (!calculoValido) {
        aplicarParada(Estado::Fallo);
        Serial.println("ERROR calculo PID; PID detenido");
    } else if (!Planta::escribirActuador(output)) {
        aplicarParada(Estado::Fallo);
        Serial.println("ERROR actuador; PID detenido");
    }
}

void LazoPID::atenderSerial() {
    comandos_.actualizar();
    comandos_.publicarTelemetria(input_, output_, nombreEstado());
}

bool LazoPID::leerEntrada() {
    double medicion = NAN;
    const bool valida = Planta::leerSensor(medicion) && isfinite(medicion) &&
                        medicion >= Config::MEDICION_MIN && medicion <= Config::MEDICION_MAX;
    input_ = valida ? medicion : NAN;
    return valida;
}

bool LazoPID::aplicarParada(Estado destino) {
    pid_.SetMode(ModoPID::Manual);
    output_ = Config::SALIDA_SEGURA;
    const bool aplicada = Planta::aplicarSalidaSegura();
    estado_ = aplicada ? destino : Estado::Fallo;
    if (!aplicada) Serial.println("ERROR salida segura no confirmada");
    return aplicada;
}

bool LazoPID::arrancar() {
    if (estado_ == Estado::Activo) return true;
    if (!sistemaListo_ || !leerEntrada()) {
        aplicarParada(Estado::Fallo);
        return false;
    }
    if (!aplicarParada(Estado::Parado)) return false;
    if (!pid_.SetMode(ModoPID::Automatico)) {
        estado_ = Estado::Fallo;
        Serial.println("ERROR parametros PID");
        return false;
    }
    estado_ = Estado::Activo;
    ultimoMuestreo_ = millis();
    return true;
}

bool LazoPID::detener() {
    return aplicarParada(sistemaListo_ ? Estado::Parado : Estado::Fallo) && sistemaListo_;
}

const char* LazoPID::nombreEstado() const {
    switch (estado_) {
        case Estado::Parado: return "PARADO";
        case Estado::Activo: return "ACTIVO";
        default: return "FALLO";
    }
}
