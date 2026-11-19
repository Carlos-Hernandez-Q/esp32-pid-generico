#pragma once

#include <stdint.h>

enum class DireccionPID { Directa, Inversa };
enum class ModoPID { Manual, Automatico };

// PID discreto para una medición y un mando. Las tres variables se comparten
// por referencia y deben existir mientras se use el controlador.
class PIDControl {
public:
    using RelojMillis = uint32_t (*)();

    PIDControl(double& input, double& output, double& setpoint,
               double kp, double ki, double kd, DireccionPID direccion);
    PIDControl(const PIDControl&) = delete;
    PIDControl& operator=(const PIDControl&) = delete;

    bool SetOutputLimits(double minimo, double maximo); // Configurar en modo manual.
    bool SetDerivativeFilter(double segundos);          // Configurar en modo manual. 0 quita el filtro.
    bool SetClock(RelojMillis reloj);                    // Conecta un reloj en ms en modo manual.
    bool SetTunings(double kp, double ki, double kd);     // Admite cambios durante la regulación.
    bool SetMode(ModoPID modo);                         // Al activar, toma la primera medición y el mando inicial.
    bool Compute();                                     // Usa el reloj configurado. Una llamada por muestra.
    bool Compute(double intervaloSegundos);             // Alternativa cuando el programa mide su propio intervalo.

    double GetKp() const { return kp_; }
    double GetKi() const { return ki_; }
    double GetKd() const { return kd_; }

private:
    double calcularPendiente(double dt) const;

    const double& input_;
    double& output_;
    const double& setpoint_;
    const double signo_;
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double minimo_ = 0.0;
    double maximo_ = 0.0;
    double filtroSegundos_ = 0.0;
    double mandoBase_ = 0.0;
    double integral_ = 0.0;
    double medicionAnterior_ = 0.0;
    double derivadaFiltrada_ = 0.0;
    RelojMillis reloj_ = nullptr;
    uint32_t ultimoCalculoMs_ = 0;
    bool gananciasValidas_ = false;
    bool activo_ = false;
};
