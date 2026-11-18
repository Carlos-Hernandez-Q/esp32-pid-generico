#pragma once

#include <Arduino.h>
#include <PIDControl.h>

// Consola opcional: el controlador tambien puede usarse sin comunicacion serial.
class ComandosPID {
public:
    ComandosPID(Stream& puerto, PIDControl& pid, double& referencia, double referenciaMin,
                double referenciaMax, uint32_t periodoTelemetriaMs,
                bool (*arrancar)(), bool (*detener)());
    void iniciar();
    void actualizar();
    void publicarTelemetria(double medicion, double salida, const char* estado);

private:
    void procesarLinea();
    void recibir();
    Stream& puerto_;
    PIDControl& pid_;
    double& referencia_;
    const double referenciaMin_;
    const double referenciaMax_;
    const uint32_t periodoTelemetriaMs_;
    bool (*arrancar_)();
    bool (*detener_)();
    uint32_t ultimaTelemetria_ = 0;
    char linea_[80] = {};
    size_t usados_ = 0;
    bool descartar_ = false;
};
