# Firmware del robot

Código C++ del microcontrolador de cada AttaBot. Un robot con este firmware
levanta WiFi, se anuncia a la estación base, obedece los comandos que le llegan
por UDP y ejecuta por su cuenta la navegación, la evasión de obstáculos y las
conductas de enjambre.

| carpeta | qué es |
|---|---|
| `AttaBot/` | el firmware normal, el que va en todos los robots |
| `HW_Test/` | diagnóstico de hardware sin WiFi — ver `Docs/HW_Test.md` |
| `IR_Calibration/` | calibración de los sensores infrarrojos |

---

## Qué hace falta

**Hardware:** ESP32 (módulo clásico, no S2/S3), dos motores DC con encoder y
puente H, dos sensores infrarrojos laterales, un APDS9960 al frente (proximidad y
color), una IMU ICM-20948, un anillo de LEDs y un marcador ArUco en la tapa.
El diagrama electrónico y la lista de materiales están en `Docs/`.

**Software:** el Arduino IDE (por su `arduino-cli` incorporado y su gestor de
librerías) o el `arduino-cli` suelto. Python 3 para `OTA.py`.

**Librerías**, con las versiones contra las que compila hoy:

| componente | versión |
|---|---|
| ESP32 Arduino core | 3.3.11 |
| FastLED | 3.10.5 |
| Adafruit APDS9960 | 1.3.1 |
| SparkFun ICM-20948 | 1.3.2 |

Estas versiones importan más de lo habitual: varias de estas librerías tienen
requisitos que **no se ven al compilar**. El sketch construye igual y el robot
arranca igual, y la falla aparece recién al moverlo. Las tres trampas conocidas
están en [Restricciones](#restricciones-que-hay-que-respetar), al final.

**Red:** los robots se conectan a un AP con el SSID y la contraseña que declara
`AttaBot/AttaBot.ino` (constantes `ssid` y `password`), y escuchan UDP en el
puerto 6060. Cada robot necesita IP fija en el router, y esa IP se anota en
`Base/configSystem.json` para que la base sepa a quién le habla.

---

## Compilar y subir

### La primera vez en cada robot: por USB

Obligatorio, porque `partitions.csv` cambia la tabla de particiones y eso no se
puede hacer por OTA — el OTA escribe en el slot de aplicación, nunca en la tabla.

```sh
cd Controller/AttaBot
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs \
    --build-path build --upload --port /dev/ttyUSB0 .
arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200
```

`arduino-cli board list` dice en qué puerto quedó el robot. Si se usa el que
viene dentro del Arduino IDE, el binario está en
`<instalación del IDE>/resources/app/lib/backend/resources/arduino-cli`.

> **`PartitionScheme=min_spiffs` no es decorativo.** `partitions.csv` le da al
> sketch un slot de 1.97 MB en vez de los 1.31 MB del layout por defecto. Sin
> declararlo, `arduino-cli` compila igual pero mide contra el slot equivocado y
> reporta 93 % de uso cuando el real es 62 %.

### Después: por aire

`OTA.py` compila y sube por WiFi en un solo paso, usando el `arduino-cli` del
IDE:

```sh
cd Controller/AttaBot
python3 OTA.py --build          # compila y sube a todos los robots
python3 OTA.py --build 101 104  # solo a .101 y .104
python3 OTA.py 101              # sube lo ya compilado
```

Antes de subir verifica que el `.bin` sea más nuevo que **todos** los `.ino` y
`.h` del sketch. Un OTA que sube un binario viejo es indistinguible de uno que
funciona, hasta que se prueba el robot.

### Desde VS Code

Las mismas operaciones están como tareas: `firmware: compilar y subir por USB`,
`ver puertos USB` y `monitor serie`. `vscode_setup.py` genera el
`c_cpp_properties.json` con las rutas del core de esta máquina (no se versiona,
porque son rutas absolutas).

---

## Estructura del sketch

Arduino concatena todos los `.ino` de la carpeta en una sola unidad de
traducción — primero el que se llama como la carpeta, después el resto en orden
alfabético. Por eso las constantes, las variables globales y las declaraciones
forward viven en `AttaBot.ino` y los demás archivos las ven directo, sin `extern`
ni cabeceras. La contra es que no hay encapsulación real: cualquier archivo puede
tocar cualquier global.

| archivo | qué lleva |
|---|---|
| `AttaBot.ino` | debug, pines, constantes, globales, declaraciones, ISR, `setup()` |
| `comandos.ino` | protocolo con la base y un handler por comando UDP |
| `estados.ino` | `loop()` y una función por estado de la FSM |
| `navegacion.ino` | navegación reactiva, congregación, MEET, dispersión, formaciones |
| `sensores.ino` | IR laterales, APDS9960 central, WiFi |
| `motores.ino` | PID por rueda, puente H, movimiento por distancia |
| `perifericos.ino` | LED, IMU y el EKF |
| `utils.h` | estructuras de estado y helpers inline |

El robot es una máquina de estados que consume una lista de instrucciones: los
comandos que llegan por UDP encolan instrucciones, y `loop()` ejecuta la que
toca. La lista de comandos y su sintaxis están en `Docs/comandos.md`; la
geometría de las formaciones, en `Docs/formaciones.md`.

Cada robot guarda en NVS su calibración propia (escala del giróscopo, pulsos por
revolución de los encoders, balance de blancos del sensor de color), así que dos
robots con el mismo binario no se comportan igual hasta calibrarlos. La rutina
guiada es `BASE.CALIBRATE|<id>` desde la consola de la base.

---

## Restricciones que hay que respetar

Tres detalles de las librerías rompen el robot **en silencio**: compilan bien y
la falla aparece recién al moverlo. Si algo de esto se toca, hay que verificarlo
en el arranque por serie.

### El PWM de los motores necesita el reloj APB

Los cuatro timers low-speed del LEDC **comparten un solo mux de reloj**. Los
motores van a 1 kHz / 14 bit, o sea 16.384 MHz de fuente, y eso sólo sale de APB
(80 MHz). Si otro periférico se queda con un reloj lento, los `ledcAttach()` de
los motores fallan y **las ruedas no reciben PWM: el robot no gira ni avanza**.

`setup()` deja el reloj explícito con `ledcSetClockSource(LEDC_USE_APB_CLK)`
antes de crear cualquier canal, así el reparto no se hereda de quien se attachee
primero. Si se monta un servo u otro periférico PWM:

- Dale **16 bits** de ancho de timer (pide 3.28 MHz → divisor 24 sobre APB), no
  los 10 por defecto. A 10 bits un servo pide 50 Hz × 2¹⁰ = 51.2 kHz, que desde
  APB daría divisor 1562 y no entra en el registro, así que el driver se va al
  reloj lento y arrastra a los motores.
- **No uses ESP32Servo 3.2.1**: su `attach()` pisa el ancho con el valor por
  defecto aunque la librería documente lo contrario. Sale más barato
  `ledcAttach(pin, 50, 16)` + `ledcWrite(pin, us << 16 / 20000)`.

Para verificarlo sin destapar nada, mirá el arranque por serie:

```
[3] Motores PWM: 1000Hz 14-bit (max=16383) OK — real=1000Hz
```

`FALLO`, o un `real=` distinto de 1000 Hz, es exactamente este problema.

### FastLED secuestra `radians()` — no la uses en este sketch

`FastLED.h` hace `#undef radians` y después `using fl::radians`. En el sketch
`radians()` **no es la macro de Arduino**, es esta plantilla:

```cpp
template<typename T>
constexpr inline T radians(T deg) { return deg * static_cast<T>(0.017453292519943295); }
```

La constante se castea al **tipo del argumento**. Con un ángulo entero,
`static_cast<int>(0.01745…)` es **0**, así que `radians(180)` devuelve **0** y
cualquier giro calculado desde un entero se vuelve un no-op silencioso. Con
`float` funciona bien.

El síntoma es característico porque separa los caminos por tipo: el robot
*navega* hacia el objetivo (la nav usa floats) pero *no esquiva* obstáculos
(ángulo de evasión entero), el random walk no gira y `TURN|grados` no hace nada
en ningún robot. Es fácil confundirlo con un problema de calibración por robot, y
no lo es.

Usá **`DegToRad()`** y **`DegToArc()`**, definidas al principio de `AttaBot.ino`:
toman `float` a propósito, para forzar la promoción en el call site, y usan la
constante literal sin depender de FastLED ni de `DEG_TO_RAD`.

La misma cabecera también reemplaza `abs`, `min`, `max`, `degrees` y `map`. Hoy
el firmware no usa `degrees()`, y `map()` sigue siendo la de Arduino (es función,
no macro), pero si aparece un cálculo raro con enteros, este es el primer lugar
donde mirar.

### El DMP de la IMU se activa editando la librería

`ICM_20948_USE_DMP` tiene que estar descomentado en `src/util/ICM_20948_C.h`
(línea ~34) de la librería de SparkFun. **Un update de la librería pisa esa
edición.** Sin DMP no hay yaw: `imuAvailable` queda en `false` y los giros caen
al lazo abierto por encoders, así que salen cortos o largos y la navegación
acumula error.
