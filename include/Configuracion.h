#pragma once // Evita incluir este archivo mas de una vez durante la compilacion.

#include <stdint.h> // Define uint32_t para periodos y velocidad serial.
#include <PIDControl.h> // Define el tipo DireccionPID.

// CONFIGURACION GENERAL: cambiar estos valores para el sistema fisico real.
// No hay pines, modelos de sensores ni protocolos de actuadores en este archivo.
// Los numeros siguientes son valores iniciales editables; no son una sintonizacion.
namespace Config { // Agrupa los parametros para no confundirlos con otras variables.

constexpr double KP = 0.0; // Ganancia proporcional: mando/unidad medida; aumenta la accion ante el error actual.
constexpr double KI = 0.0; // Ganancia integral: mando/(unidad medida*s); corrige error sostenido; 0 desactiva I.
constexpr double KD = 0.0; // Ganancia derivativa: mando*s/unidad medida; actua sobre cambios de medicion; 0 desactiva D.
constexpr DireccionPID DIRECCION = DireccionPID::Directa; // Directa si más mando eleva la medición. Inversa si la reduce.
constexpr uint32_t PERIODO_PID_MS = 20; // Intervalo mínimo para solicitar otra muestra, en milisegundos.
constexpr double FILTRO_D_SEG = 0.05; // Constante de tiempo del filtro derivativo, en segundos. 0 quita el filtro.

constexpr double REFERENCIA_INICIAL = 0.0; // Objetivo inicial en las mismas unidades fisicas que devuelve el sensor.
constexpr double MEDICION_MIN = 0.0; // Menor lectura fisica valida; cambiar segun el rango util del sensor/sistema.
constexpr double MEDICION_MAX = 100.0; // Mayor lectura fisica valida; tambien limita las referencias aceptadas.

constexpr double SALIDA_MIN = 0.0; // Menor mando realmente aplicable: por ejemplo 0 %, -100 % o desplazamiento angular.
constexpr double SALIDA_MAX = 100.0; // Mayor mando realmente aplicable, en las mismas unidades que SALIDA_MIN.
constexpr double SALIDA_SEGURA = 0.0; // Mando solicitado al parar/fallar; su significado lo implementa el adaptador del actuador.

constexpr uint32_t BAUDIOS = 115200; // Velocidad del puerto serial; usar la misma en el monitor o en LabVIEW.
constexpr uint32_t PERIODO_TELEMETRIA_MS = 200; // Intervalo de envio CSV; 0 desactiva el CSV y conserva los comandos.

static_assert(KP >= 0 && KI >= 0 && KD >= 0, "Ganancias negativas"); // El sentido se fija con DIRECCION.
static_assert(PERIODO_PID_MS > 0, "Periodo invalido"); // Intervalo mínimo de un milisegundo para el reloj millis().
static_assert(FILTRO_D_SEG >= 0, "Filtro invalido"); // La constante de tiempo debe ser mayor o igual a cero.
static_assert(MEDICION_MIN < MEDICION_MAX, "Rango de medicion invalido"); // Exige un intervalo fisico no vacio.
static_assert(REFERENCIA_INICIAL >= MEDICION_MIN && REFERENCIA_INICIAL <= MEDICION_MAX, "Referencia fuera de rango"); // Comprueba el objetivo inicial.
static_assert(SALIDA_MIN < SALIDA_MAX, "Rango de salida invalido"); // Exige limites de mando ordenados.
static_assert(SALIDA_SEGURA >= SALIDA_MIN && SALIDA_SEGURA <= SALIDA_MAX, "Salida segura fuera de rango"); // Comprueba el mando de parada.
static_assert(BAUDIOS > 0, "Baudios invalidos"); // Impide configurar una velocidad serial nula.

} // Termina el grupo de parámetros que usa el main.
