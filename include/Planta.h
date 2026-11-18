#pragma once

// Este es el unico contrato de hardware. No depende de servo, Sharp, bomba o motor.
// Todas las funciones deben terminar rapido: no usar delay() ni esperas largas.
namespace Planta {
bool iniciar();                       // Configurar los dispositivos reales.
bool leerSensor(double& medicion);     // Entregar una lectura valida en unidades fisicas.
bool escribirActuador(double salida);  // Aplicar el mando en las unidades de Configuracion.h.
bool aplicarSalidaSegura();            // Ejecutar la parada adecuada al dispositivo real.

}
