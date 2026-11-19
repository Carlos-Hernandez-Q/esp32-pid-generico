#pragma once

#include <ComandosPID.h>

// Operación del proyecto ESP32. El PID y el hardware se conectan desde main.cpp.
class LazoPID {
public:
    LazoPID(PIDControl& pid, double& input, double& output, double& setpoint);
    LazoPID(const LazoPID&) = delete;
    LazoPID& operator=(const LazoPID&) = delete;

    void iniciar(bool configuracionValida);
    bool leerSensor(); // Actualiza Input. Devuelve true si el PID debe calcular.
    void aplicarSalida(double output, bool calculoValido);
    void atenderSerial();

private:
    enum class Estado { Parado, Activo, Fallo };
    bool leerEntrada();
    bool arrancar();
    bool detener();
    bool aplicarParada(Estado destino);
    const char* nombreEstado() const;

    PIDControl& pid_;
    double& input_;
    double& output_;
    Estado estado_ = Estado::Parado;
    bool sistemaListo_ = false;
    uint32_t ultimoMuestreo_ = 0;
    ComandosPID comandos_;
};
