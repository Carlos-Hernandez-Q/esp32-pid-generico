# Plantilla PID general para ESP32

Esqueleto para un sistema **físico** de una variable medida y un mando. Cada
equipo implementa su sensor y actuador, y configura el lazo según su planta.
Las ganancias, unidades, dirección y periodo necesitan ajuste para cada sistema.

[Consultar el manual de uso en PDF](output/pdf/manual-pid-esp32.pdf). Incluye
la configuración, la integración del hardware, la API y los comandos seriales.

## Explicación de `main.cpp`

Para presentar el programa, seguir una vuelta del lazo. El sensor actualiza
`Input`, el PID lo compara con `Setpoint` y el actuador recibe `Output`. Al
volver a medir se comprueba el efecto del último mando sobre la planta.

### 1. Variables e instancia del PID

| Variable | Qué significa | Ejemplo para un tanque con bomba |
| --- | --- | --- |
| `Input` | La medición que escribe `Planta::leerSensor()`. Empieza como `nan` hasta obtener una lectura válida. | Nivel medido en cm. |
| `Setpoint` | El objetivo inicial viene de la configuración y puede cambiar por serial. Se expresa en las mismas unidades que `Input`. | Nivel deseado en cm. |
| `Output` | El mando que escribe el PID. Empieza en `SALIDA_SEGURA` y se expresa en las unidades que espera el actuador. | Potencia solicitada de 0 a 100 %. |

```cpp
PIDControl miPID(Input, Output, Setpoint, Config::KP, Config::KI, Config::KD, Config::DIRECCION);
```

El orden de los argumentos es entrada, salida y referencia. Se pasan por
referencia de C++, así que el sensor, la consola y el PID trabajan con las
mismas variables. Si aumentar el mando hace subir la medición, se configura
`DireccionPID::Directa`. Si la hace bajar, se configura `DireccionPID::Inversa`.

### 2. Qué hace `setup()`

1. Abre el puerto serial a la velocidad de `Config::BAUDIOS`.
2. Aplica `SetOutputLimits(SALIDA_MIN, SALIDA_MAX)` para fijar el rango de
   mando disponible. Por ejemplo, una bomba puede usar 0 a 100 % y un servo
   puede usar grados respecto a su centro. Cada equipo elige sus límites.
3. Configura la constante de tiempo del filtro derivativo con
   `SetDerivativeFilter(Config::FILTRO_D_SEG)`.
4. Llama a `Planta::iniciar()` para preparar el hardware y solicita la parada.
   El sistema queda en `PARADO` si la preparación y la parada son correctas.
   Si alguna falla, queda en `FALLO`.
5. Guarda el tiempo inicial con `millis()` y muestra las órdenes seriales.

### 3. Qué hace `loop()`

1. Si el sistema está listo y ha pasado al menos `PERIODO_PID_MS` desde que
   estuvo lista la última muestra, llama a `leerEntrada()`. Esa función pide
   la medición a `Planta::leerSensor(Input)` y verifica que sea finita y esté
   entre `MEDICION_MIN` y `MEDICION_MAX`.
2. Registra el instante en el que terminó la lectura. Resta el instante
   anterior y divide entre 1000 para obtener `dtSegundos`. Por ejemplo, si
   el periodo está en 20 ms y entre lecturas pasan 24 ms, el PID recibe
   `0.024` segundos.
3. Con el estado `ACTIVO` y una lectura válida, `miPID.Compute(dtSegundos)`
   calcula y actualiza `Output`. Solo si el cálculo tiene éxito se llama a
   `Planta::escribirActuador(Output)`. Una lectura, un cálculo o una escritura
   que fallen llevan el control a `FALLO` y solicitan la salida segura.
4. En cada vuelta, incluso entre muestras, se atienden los comandos y se
   intenta publicar la telemetría cuando corresponde. Mientras el control
   está parado, las mediciones sirven para observar el sensor.

### 4. Arranque, parada y fallos

- `arrancarPID()` pide una primera medición válida y confirma que el adaptador
  aceptó la orden de salida segura. Luego activa el modo automático. El PID
  toma esa medición como punto inicial de la derivada y el mando seguro como
  base del cálculo.
- `detenerPID()` llama a `aplicarParada()`. Esa función pasa a modo manual,
  registra `SALIDA_SEGURA` en `Output` y llama a `Planta::aplicarSalidaSegura()`.
  La forma de detener una bomba, un motor o un servo la decide su adaptador.
- `nombreEstado()` traduce el estado a `PARADO`, `ACTIVO` o `FALLO` para el
  CSV. Después de corregir un fallo durante la operación se puede enviar
  `START` de nuevo. Si falló la inicialización, se corrige y se reinicia.

## Librerías del proyecto

| Librería | Responsabilidad | Archivos |
| --- | --- | --- |
| `PIDControl` | Calcula un mando a partir de la medición, la referencia y el intervalo en segundos. Aplica los límites configurados. | `lib/PIDControl/src/PIDControl.h` y `PIDControl.cpp` |
| `ComandosPID` | Recibe órdenes por serial y publica el CSV. Trabaja con la misma instancia de PID y el mismo `Setpoint` del `main`. | `lib/ComandosPID/src/ComandosPID.h` y `ComandosPID.cpp` |

Para seguir el cálculo en el código, lee primero
[`Compute(dt)`](lib/PIDControl/src/PIDControl.cpp). Ahí se ve cómo se obtiene
el mando a partir de P, I, D y los límites del actuador.

En `PIDControl`, P responde al error actual y la integral I acumula el error
con el tiempo real entre muestras. La integral solo crece hasta el límite
hacia el que empujaría el mando. Si el incremento ayuda a salir del límite,
se permite. Esta protección contra saturación se conoce como *anti-windup*.
[Rundqwist, *Anti-Reset Windup for PID Controllers*](https://lup.lub.lu.se/search/files/11244316/rund90t.pdf)

El término D se calcula a partir del cambio de la **medición**. Así un cambio
en `Setpoint` no produce un salto por la derivada. El filtro de primer orden
suaviza la contribución de D cuando hay ruido. `FILTRO_D_SEG = 0` quita el
filtro y `KD = 0` desactiva el término derivativo completo. El cálculo suma
los tres términos al mando base y limita el resultado al rango del actuador.
[Åström y Murray, *Feedback Systems*, capítulo de control PID](https://www.cds.caltech.edu/~murray/books/AM05/pdf/am08-pid_19Jul11.pdf)

En `ComandosPID`, `SP` valida y modifica `Setpoint`, `PID` actualiza las tres
ganancias con `SetTunings()` y `START` y `STOP` llaman a las funciones de
control del `main`. La consola recibe los datos del lazo para armar el CSV.

## Qué debe completar cada equipo

En `Planta.cpp`:

1. `iniciar()`: preparar los dispositivos reales y devolver si están listos.
2. `leerSensor(double& medicion)`: escribir una lectura válida en unidades
   físicas y devolver `true`; ante un fallo, devolver `false`.
3. `escribirActuador(double salida)`: convertir el mando al protocolo del
   dispositivo real, aplicarlo y devolver si el adaptador pudo hacerlo.
4. `aplicarSalidaSegura()`: ejecutar la parada adecuada al equipo. Si basta
   enviar un valor, puede llamar a `escribirActuador(Config::SALIDA_SEGURA)`.

Las funciones pendientes devuelven `false`. **La plantilla compila, pero no
habilita el PID hasta implementar el hardware.** No se deben sustituir esos
retornos por `true` sin realizar las operaciones correspondientes.

Declarar los pines y parámetros específicos de los dispositivos en `Planta.cpp`
o en archivos propios de sus adaptadores. Incluir allí sus librerías y añadir
las dependencias a `platformio.ini` cuando haga falta.

En `include/Configuracion.h`, cada parámetro está comentado en su propia
línea. Ajustar ganancias, referencia, rangos, dirección, periodo, filtro
derivativo y salida segura. La referencia y la medición usan las mismas
unidades. El mando tiene sus propias unidades según el dispositivo.

| Equipo | Medición posible | Mando posible |
| --- | --- | --- |
| Sharp + servo | Distancia en cm | Grados respecto al centro del servo |
| Nivel de tanque + bomba | Nivel en cm | Porcentaje de potencia del driver |
| Encoder + motor DC | Velocidad en rpm | Porcentaje con signo para un puente H |
| Sensor de velocidad + brushless | Velocidad en rpm | Mando al ESC según su protocolo |

El PID entrega un número; el adaptador determina cómo llevarlo al hardware.
Un brushless necesita su ESC/driver, su protocolo y su procedimiento de armado.
Los límites configurados deben coincidir con los realmente aplicables. Evitar
recortes adicionales que la integral del PID no conozca.

Las funciones de hardware deben ser breves, sin `delay()` ni esperas largas.
Para un sensor asíncrono se puede devolver la última muestra, comprobando
que siga vigente. El intervalo real puede aumentar por el tiempo de lectura
y otras tareas del lazo. Medir ese intervalo permite usarlo en el cálculo
del PID. La rapidez con la que responde el lazo sigue dependiendo del sensor
y de la frecuencia con la que se ejecuta `loop()`.

## Compilar y usar

Abrir esta carpeta como proyecto de PlatformIO. La placa configurada es un
ESP32 DevKit clásico (`esp32dev`); cambiar `board` si se usa otra placa.

```text
pio run -e esp32
pio run -e esp32 -t upload
pio device monitor -b 115200
```

Una vez implementado el hardware, enviar comandos con salto de línea y punto
decimal:

```text
PID 1.0 0.4 0.0
SP 25
START
STOP
```

Los números del bloque ilustran el formato de los comandos; no son ganancias
ni una referencia recomendadas para una planta concreta. En código, la
referencia se cambia asignando `Setpoint` y las ganancias con `miPID.SetTunings()`.

El control arranca detenido. `START` valida el sensor, pide la salida segura y
habilita la regulación; `STOP` vuelve a manual y pide la parada. Un fallo de
lectura o escritura detiene el PID. Corregir la causa y enviar `START` para
reactivarlo; si falló la inicialización, corregir y reiniciar el programa.

La consola envía CSV con columnas:

```text
ms,estado,referencia,medicion,salida,kp,ki,kd
```

Las respuestas comienzan con `OK` o `ERROR`. `nan` identifica una medición
inválida. La salida es el mando solicitado: no confirma un movimiento físico.
En un fallo, tampoco garantiza que el dispositivo haya podido alcanzar su
estado seguro. La salida segura inicial se solicita incluso si la preparación
del hardware falla; el adaptador debe tolerar ese caso y devolver su resultado.

Este protocolo requiere adaptar el VI anterior de LabVIEW. Los cambios por
serial se pierden al reiniciar; editar `Configuracion.h` para fijar los de arranque.
La telemetría se omite si no cabe en el buffer del puerto. En un `Stream` que
no implemente `availableForWrite()` con capacidad positiva no se envía CSV.

Para arrancar desde código, llamar a `arrancarPID()` tras inicializar el hardware.

## Validación

Se compiló correctamente el entorno `esp32` con `pio run -e esp32`. En
pruebas numéricas de C++ se verificaron los límites, los dos sentidos de
control, la recuperación tras saturación, el intervalo variable, el filtro
derivativo y el rechazo de entradas no válidas. La conexión, calibración y
sintonización quedan pendientes de prueba con el sistema real de cada equipo.
