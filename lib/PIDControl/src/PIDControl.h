#pragma once

enum class DireccionPID { Directa, Inversa };
enum class ModoPID { Manual, Automatico };

// PID discreto para una medición y un mando. Las tres variables se comparten
// por referencia y deben existir mientras se use el controlador.
class PIDControl {
public:
    PIDControl(double& input, double& output, double& setpoint,
               double kp, double ki, double kd, DireccionPID direccion);
    PIDControl(const PIDControl&) = delete;
    PIDControl& operator=(const PIDControl&) = delete;

    bool SetOutputLimits(double minimo, double maximo); // Configurar en modo manual.
    bool SetDerivativeFilter(double segundos);          // Configurar en modo manual. 0 quita el filtro.
    bool SetTunings(double kp, double ki, double kd);     // Admite cambios durante la regulación.
    bool SetMode(ModoPID modo);                         // Al activar, toma la primera medición y el mando inicial.
    bool Compute(double intervaloSegundos);             // Escribe Output solamente si el cálculo resulta válido.

    double GetKp() const { return kp_; }
    double GetKi() const { return ki_; }
    double GetKd() const { return kd_; }

private:
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
    bool gananciasValidas_ = false;
    bool activo_ = false;
};
