#include "Planta.h"        // Funciones de hardware que se llaman desde el main.
#include "Configuracion.h" // Permite conocer las unidades y limites del mando.

// IMPLEMENTAR AQUI EL HARDWARE REAL DE CADA PROYECTO.
// Los pines y parametros propios del sensor/actuador se declaran en este archivo.
// Se pueden incluir aqui las librerias del fabricante o del dispositivo elegido.
// No se generan lecturas ni movimientos de ejemplo: falta conectar estas funciones.

bool Planta::iniciar() {
    // TODO: configurar el sensor real: pines, ADC, encoder, bus I2C/SPI, etc.
    // TODO: configurar el actuador real: servo, PWM + driver, puente H o ESC.
    // Si un dispositivo requiere armado/espera, gestionar su estado sin bloquear el lazo.
    // Devolver true cuando el hardware este preparado; false si hay un fallo.
    return false; // Pendiente de implementacion; impide habilitar un hardware inexistente.
}

bool Planta::leerSensor(double& medicion) {
    // TODO: leer el sensor real y convertir a la variable controlada: cm, rpm, grados, etc.
    // TODO: validar la muestra y, si se almacena en segundo plano, comprobar su antiguedad.
    // Asignar el resultado a 'medicion' y devolver true solo cuando la lectura sea valida.
    // Ante un fallo de lectura, devolver false; el controlador solicitara la salida segura.
    (void)medicion; // Evita una advertencia mientras todavia no se implementa la lectura.
    return false; // No se entrega ningun valor ficticio al PID.
}

bool Planta::escribirActuador(double salida) {
    // TODO: convertir 'salida' al protocolo real del actuador y aplicar el mando.
    // Ejemplos de conversion: porcentaje a duty; desplazamiento a pulso de servo;
    // signo y magnitud a puente H; mando a protocolo del ESC del motor brushless.
    // Respetar SALIDA_MIN y SALIDA_MAX; si el rango real cambia, actualizar la configuracion.
    // Devolver true si el adaptador pudo aplicar el mando; false si detecta un fallo.
    (void)salida; // Evita una advertencia mientras todavia no se implementa la escritura.
    return false; // No se afirma haber accionado un equipo que aun no esta implementado.
}

bool Planta::aplicarSalidaSegura() {
    // TODO: implementar la parada real: bomba apagada, motor detenido, centro del servo, etc.
    // Si basta un mando numerico, usar: return escribirActuador(Config::SALIDA_SEGURA);
    // Si requiere otra accion (deshabilitar driver, frenar o enviar parada al ESC), hacerla aqui.
    return false; // Cambiar cuando el adaptador realmente pueda aplicar el estado seguro.
}
