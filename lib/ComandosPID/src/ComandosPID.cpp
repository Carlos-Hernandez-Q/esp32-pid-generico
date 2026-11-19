#include "ComandosPID.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <utility>

namespace {
bool leerNumero(char*& cursor, double& valor) {
    char* fin = nullptr;
    valor = strtod(cursor, &fin);
    if (fin == cursor || !isfinite(valor) ||
        (*fin != '\0' && *fin != ' ' && *fin != '\t')) return false;
    cursor = fin;
    return true;
}

bool finDeComando(const char* cursor) {
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    return *cursor == '\0';
}
}

ComandosPID::ComandosPID(Stream& puerto, PIDControl& pid, double& referencia, double referenciaMin,
                         double referenciaMax, uint32_t periodoTelemetriaMs,
                         std::function<bool()> arrancar, std::function<bool()> detener)
    : puerto_(puerto), pid_(pid), referencia_(referencia), referenciaMin_(referenciaMin),
      referenciaMax_(referenciaMax), periodoTelemetriaMs_(periodoTelemetriaMs),
      arrancar_(std::move(arrancar)), detener_(std::move(detener)) {}

void ComandosPID::iniciar() {
    puerto_.println("Comandos: START | STOP | SP valor | PID kp ki kd");
    puerto_.println("ms,estado,referencia,medicion,salida,kp,ki,kd");
}

void ComandosPID::procesarLinea() {
    if (strcmp(linea_, "START") == 0) {
        puerto_.println(arrancar_() ? "OK START" : "ERROR hardware/sensor/actuador");
        return;
    }
    if (strcmp(linea_, "STOP") == 0) {
        puerto_.println(detener_() ? "OK STOP" : "ERROR hardware/salida segura");
        return;
    }
    if (strncmp(linea_, "SP ", 3) == 0) {
        char* cursor = linea_ + 3;
        double nuevoSetpoint;
        if (leerNumero(cursor, nuevoSetpoint) && finDeComando(cursor) &&
            nuevoSetpoint >= referenciaMin_ && nuevoSetpoint <= referenciaMax_) {
            referencia_ = nuevoSetpoint;
            puerto_.println("OK SP");
            return;
        }
    } else if (strncmp(linea_, "PID ", 4) == 0) {
        char* cursor = linea_ + 4;
        double kp, ki, kd;
        if (leerNumero(cursor, kp) && leerNumero(cursor, ki) && leerNumero(cursor, kd) &&
            finDeComando(cursor) && pid_.SetTunings(kp, ki, kd)) {
            puerto_.println("OK PID");
            return;
        }
    }
    puerto_.println("ERROR usar START, STOP, SP valor o PID kp ki kd");
}

void ComandosPID::recibir() {
    uint8_t presupuesto = 32; // Limita el trabajo serial de cada vuelta del lazo.
    while (presupuesto-- > 0 && puerto_.available() > 0) {
        const char c = static_cast<char>(puerto_.read());
        if (c == '\r') continue;
        if (c == '\n') {
            if (descartar_) puerto_.println("ERROR linea demasiado larga o invalida");
            else if (usados_ > 0) {
                linea_[usados_] = '\0';
                procesarLinea();
            }
            usados_ = 0;
            descartar_ = false;
        } else if (!descartar_) {
            if (usados_ + 1 < sizeof(linea_) && (c == '\t' || (c >= 32 && c <= 126))) {
                linea_[usados_++] = c;
            } else descartar_ = true;
        }
    }
}

void ComandosPID::publicarTelemetria(double medicion, double salida, const char* estado) {
    const uint32_t ahora = millis();
    if (periodoTelemetriaMs_ == 0 || ahora - ultimaTelemetria_ < periodoTelemetriaMs_) return;
    ultimaTelemetria_ = ahora;
    char linea[160];
    const int n = snprintf(linea, sizeof(linea), "%lu,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        static_cast<unsigned long>(ahora), estado, referencia_, medicion, salida,
        pid_.GetKp(), pid_.GetKi(), pid_.GetKd());
    if (n > 0 && n < static_cast<int>(sizeof(linea)) && puerto_.availableForWrite() >= n) {
        puerto_.write(reinterpret_cast<const uint8_t*>(linea), n);
    }
}

void ComandosPID::actualizar() {
    recibir();
}
