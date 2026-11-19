# PID para ESP32 con LDR y LED

El punto de partida es una práctica con hardware real. Un LDR mide la luz,
el PID calcula cuánto aumentar o reducir el brillo, y el LED ilumina el LDR
para cerrar el lazo.

**La configuración y los puntos de adaptación están en un solo archivo.**
[Abrir `src/main.cpp`](src/main.cpp) · [Guía en PDF, línea por línea](output/pdf/manual-pid-esp32.pdf)

El PDF explica las 71 líneas del programa y muestra los cambios para usar un
TMP36 por ADC, un BH1750 por I2C o un actuador con driver PWM.

## Conectar y usar el ejemplo

| Componente | Conexión en el ESP32 DevKit clásico |
| --- | --- |
| LDR | Entre 3V3 y el nodo de medición. Conecta una resistencia de 10 kΩ entre el nodo y GND, y lleva el nodo a GPIO34. |
| LED | GPIO18 a una resistencia de 330 Ω y luego al ánodo del LED. Conecta el cátodo a GND. |

Apunta el LED hacia el LDR. Con el divisor conectado de esta forma, más luz
eleva la lectura. El control arranca automáticamente con el LED en PWM 50.

```text
pio run
pio run -t upload
pio device monitor
```

El monitor funciona a 115200 baudios y muestra `lectura_cruda,input,setpoint,output`.
`output` es el mando del PID antes de redondearlo al entero que recibe el PWM.

## Seguir el cálculo en el main

| Variable | En esta práctica |
| --- | --- |
| `Input` | Lectura del ADC de 10 bits recortada a **500–1000**. Si el LDR lee 420, el PID recibe 500. Si lee 1010, recibe 1000. |
| `Setpoint` | Objetivo inicial de **750 cuentas del ADC**. |
| `Output` | Mando limitado a **50–200** por `SetOutputLimits()`. Se envía a un PWM de 8 bits con `analogWrite()`. |

`setup()` prepara el hardware, establece los límites de salida y activa el
PID con una primera lectura real. En `loop()` se repite esta secuencia cada
50 ms aproximadamente.

```cpp
const double lectura = leerSensor();
Input = constrain(lectura, ENTRADA_MIN, ENTRADA_MAX);
if (!miPID.Compute()) return;
escribirActuador(Output);
```

El recorte de la entrada conserva una lectura dentro del rango que usa el
PID. Para detectar fallas de un sensor diferente, valida su señal en el
código que conecte ese equipo.

## Adaptarlo a otro sensor o actuador

1. **Arriba de `main.cpp`.** Cambia pines, ganancias, dirección, periodo,
   `Setpoint` y los límites. La lectura y el objetivo deben usar la misma
   unidad. Los límites de salida deben usar las unidades del mando.
2. **`iniciarHardware()`.** Configura los pines y periféricos del equipo
   que vas a usar. El ejemplo contiene la configuración real del ADC y PWM.
3. **`leerSensor()`.** Sustituye la lectura del LDR por la lectura y
   conversión del sensor elegido.
4. **`escribirActuador()`.** Sustituye la escritura PWM del LED por la
   conversión que requiera tu actuador o su driver. Define además el mando
   inicial adecuado en `Output`.

| Otro equipo | Lectura y referencia | Mando posible |
| --- | --- | --- |
| Sensor de nivel y bomba | Altura en cm | Porcentaje para el driver de la bomba |
| Encoder y motor | Velocidad en rpm | Porcentaje para el controlador del motor |
| Sensor Sharp y servo | Distancia en cm | Desplazamiento angular respecto al centro |

Usa `DireccionPID::Directa` si al subir el mando sube la medición, e
`Inversa` si baja. Las ganancias del LDR son un punto de partida para
esta práctica. Si el PWM llega a 50 o 200 y la lectura sigue lejos de
750, revisa la luz ambiente y la posición del LED respecto al sensor.

## Qué hace la librería

`PIDControl` recibe `Input`, `Output`, `Setpoint`, las tres ganancias y
la dirección. `SetClock()` conecta el reloj, `SetMode()` activa o pausa
el cálculo y `Compute()` actualiza `Output` usando el tiempo entre muestras.
La librería limita el mando y evita que la integral siga empujando hacia
un límite saturado. La derivada sobre la medición puede filtrarse con
`SetDerivativeFilter()`. En esta práctica `KD` se deja en cero.

`platformio.ini` define la placa y usa un entorno ESP32. Cambia ese archivo
si usas otro modelo de placa. La práctica usa
[la API ADC de Espressif](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html)
y [la API PWM de Arduino-ESP32 2.0.17](https://github.com/espressif/arduino-esp32/blob/2.0.17/cores/esp32/esp32-hal-ledc.c).
