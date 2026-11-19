#include "PIDControl.h"
#include <algorithm>
#include <cmath>

namespace {
double limitar(double valor, double minimo, double maximo) {
    return std::max(minimo, std::min(maximo, valor));
}

double limitarPasoIntegral(double mando, double paso, double minimo, double maximo) {
    // Con cero siempre permitido, la integral solo cambia en la dirección del error.
    const double margenInferior = std::min(0.0, minimo - mando);
    const double margenSuperior = std::max(0.0, maximo - mando);
    return limitar(paso, margenInferior, margenSuperior);
}
}

PIDControl::PIDControl(double& input, double& output, double& setpoint,
                       double kp, double ki, double kd, DireccionPID direccion)
    : input_(input), output_(output), setpoint_(setpoint),
      signo_(direccion == DireccionPID::Directa ? 1.0 : -1.0) {
    SetTunings(kp, ki, kd);
}

bool PIDControl::SetClock(RelojMillis reloj) {
    if (activo_ || !reloj) return false;
    reloj_ = reloj;
    return true;
}

bool PIDControl::Compute() {
    if (!activo_ || !reloj_) return false;
    const uint32_t ahora = reloj_();
    const uint32_t intervaloMs = ahora - ultimoCalculoMs_;
    if (!Compute(intervaloMs / 1000.0)) return false;
    ultimoCalculoMs_ = ahora;
    return true;
}

bool PIDControl::Compute(double dt) {
    if (!activo_ || !std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(input_) || !std::isfinite(setpoint_)) return false;

    const double error = signo_ * (setpoint_ - input_);
    const double proporcional = kp_ * error;
    const double pendiente = calcularPendiente(dt);
    const double derivativo = -signo_ * kd_ * pendiente;

    // El mando suma la base y las acciones P, I y D. I cambia según el error y dt.
    const double mandoActual = mandoBase_ + proporcional + integral_ + derivativo;
    const double pasoIntegral = ki_ * error * dt;
    if (!std::isfinite(mandoActual) || !std::isfinite(pasoIntegral)) return false;

    const double pasoPermitido = limitarPasoIntegral(mandoActual, pasoIntegral, minimo_, maximo_);
    const double nuevaIntegral = integral_ + pasoPermitido;
    const double nuevoMando = mandoActual + pasoPermitido;
    if (!std::isfinite(nuevaIntegral) || !std::isfinite(nuevoMando)) return false;

    integral_ = nuevaIntegral;
    derivadaFiltrada_ = pendiente;
    medicionAnterior_ = input_;
    output_ = limitar(nuevoMando, minimo_, maximo_);
    return true;
}

double PIDControl::calcularPendiente(double dt) const {
    if (kd_ == 0.0) return 0.0;
    const double intervalo = filtroSegundos_ + dt;
    if (!std::isfinite(intervalo)) return NAN;

    // Sin filtro, queda el cambio de la medición dividido entre dt.
    return (filtroSegundos_ / intervalo) * derivadaFiltrada_ +
           (input_ - medicionAnterior_) / intervalo;
}

bool PIDControl::SetOutputLimits(double minimo, double maximo) {
    if (activo_ || !std::isfinite(minimo) || !std::isfinite(maximo) || minimo >= maximo) {
        return false;
    }
    minimo_ = minimo;
    maximo_ = maximo;
    return true;
}

bool PIDControl::SetDerivativeFilter(double segundos) {
    if (activo_ || !std::isfinite(segundos) || segundos < 0.0) return false;
    filtroSegundos_ = segundos;
    return true;
}

bool PIDControl::SetTunings(double kp, double ki, double kd) {
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) ||
        kp < 0.0 || ki < 0.0 || kd < 0.0) return false;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    if (ki_ == 0.0) integral_ = 0.0;
    if (kd_ == 0.0) derivadaFiltrada_ = 0.0;
    gananciasValidas_ = true;
    return true;
}

bool PIDControl::SetMode(ModoPID modo) {
    if (modo == ModoPID::Manual) {
        activo_ = false;
        return true;
    }
    if (activo_) return true;
    if (!gananciasValidas_ || minimo_ >= maximo_ || !std::isfinite(input_) ||
        !std::isfinite(output_) || !std::isfinite(setpoint_)) return false;

    // La primera muestra fija el origen de la derivada. El mando seguro sirve de base.
    mandoBase_ = limitar(output_, minimo_, maximo_);
    medicionAnterior_ = input_;
    derivadaFiltrada_ = 0.0;
    integral_ = 0.0;
    if (reloj_) ultimoCalculoMs_ = reloj_();
    activo_ = true;
    return true;
}
