#include "PIDControl.h"
#include <algorithm>
#include <cmath>

namespace {
double limitar(double valor, double minimo, double maximo) {
    return std::max(minimo, std::min(maximo, valor));
}
}

PIDControl::PIDControl(double& input, double& output, double& setpoint,
                       double kp, double ki, double kd, DireccionPID direccion)
    : input_(input), output_(output), setpoint_(setpoint),
      signo_(direccion == DireccionPID::Directa ? 1.0 : -1.0) {
    SetTunings(kp, ki, kd);
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
    activo_ = true;
    return true;
}

bool PIDControl::Compute(double dt) {
    if (!activo_ || !std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(input_) || !std::isfinite(setpoint_)) return false;

    const double error = signo_ * (setpoint_ - input_);
    const double proporcional = kp_ * error;
    double nuevaDerivada = 0.0;
    if (kd_ > 0.0) {
        // Filtro de primer orden de la derivada medida. Con tau=0 queda ΔInput/dt.
        const double denominador = filtroSegundos_ + dt;
        if (!std::isfinite(denominador)) return false;
        nuevaDerivada = (filtroSegundos_ / denominador) * derivadaFiltrada_ +
                        (input_ - medicionAnterior_) / denominador;
    }
    const double derivativo = -signo_ * kd_ * nuevaDerivada;

    const double mandoSinIncremento = mandoBase_ + proporcional + integral_ + derivativo;
    double incrementoIntegral = ki_ * error * dt;
    if (!std::isfinite(mandoSinIncremento) || !std::isfinite(incrementoIntegral)) return false;

    // Solo permite integrar hasta el límite hacia el que empuja el incremento.
    // Un incremento en el sentido opuesto ayuda a recuperar margen de mando.
    if (incrementoIntegral > 0.0) {
        incrementoIntegral = std::min(incrementoIntegral, std::max(0.0, maximo_ - mandoSinIncremento));
    } else if (incrementoIntegral < 0.0) {
        incrementoIntegral = std::max(incrementoIntegral, std::min(0.0, minimo_ - mandoSinIncremento));
    }

    const double nuevaIntegral = integral_ + incrementoIntegral;
    const double mando = mandoSinIncremento + incrementoIntegral;
    if (!std::isfinite(nuevaIntegral) || !std::isfinite(mando)) return false;

    integral_ = nuevaIntegral;
    derivadaFiltrada_ = nuevaDerivada;
    medicionAnterior_ = input_;
    output_ = limitar(mando, minimo_, maximo_);
    return true;
}
