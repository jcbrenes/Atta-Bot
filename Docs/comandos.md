# Comandos de la Base — referencia

Todo lo que se le puede escribir a la consola de `AttaBot_Base.py` o a la caja de
comandos de la GUI. Las dos caras usan **el mismo despachador**, así que lo de
acá vale igual en ambas.

> La lista viva está en `_ROBOT_CMDS` y `_BASE_CMDS`, al principio de
> `Base/AttaBot_Base.py`. Son la fuente única del despachador, del autocompletado
> y de `HELP`. Si agregás un comando ahí, actualizá también esta hoja.

---

## La gramática

```
DESTINO.VERBO|arg|arg
```

| destino | qué hace | ejemplo |
|---|---|---|
| `BASE` | lo ejecuta la Base (orquesta, consulta, calibra) | `BASE.CALIBRATE\|1` |
| `<id>` | se envía tal cual por UDP a ese robot | `1.MOVE\|500` |
| `BROADCAST` | se envía tal cual a todos los robots | `BROADCAST.DISPERSE\|600` |

Los argumentos van **siempre con `|`**, nunca con espacios.

**Por qué `BASE.` y no el id del líder:** `CONGREGATION` y `FORMATION` existen
*también* como comandos del firmware. `1.CONGREGATION` sería ambiguo — ¿lo
orquesta la Base o se lo mando crudo al robot? Con `BASE.` no hay colisión
posible.

### En la consola

- **TAB** autocompleta: sin punto ofrece destinos, después del punto ofrece los
  verbos que corresponden a ese destino, y después de un `|` muestra la forma de
  los argumentos.
- **`HELP`** sola (sin prefijo) lista todo. **`BASE.HELP|MOVE`** explica uno.
- Flechas ↑↓ para el historial, que **persiste entre corridas** en
  `~/.attabot_history`.
- **`BREAK`** termina la corrida y cierra los logs y el video correctamente.

### Formas viejas

`CALIBRATE.1`, `FORMATION.linea 1 300`, `STATUS.x` y `CONGREGATION.1`
**siguen funcionando**, avisando una vez por verbo. Van a desaparecer; usá la
forma nueva.

`GOTO.1 1200 850` **ya no**: ese atajo era exactamente `1.GT|1200|850` con un
mensaje de más, así que se retiró junto con los alias del firmware.

---

## Comandos de la Base

| comando | qué hace |
|---|---|
| `BASE.HELP` | lista todo |
| `BASE.HELP\|<verbo>` | detalle de uno |
| `BASE.STATUS` | markers detectados y pose de cada robot |
| `BASE.CALIBRATE\|<id>` | calibración guiada de yaw y PPR |
| `BASE.CONGREGATION\|<líder>[\|<esp_mm>]` | congregación con líder (esp. 300) |
| `BASE.FORMATION\|<figura>\|<líder>[\|<esp_mm>]` | formación geométrica |
| `BASE.OCCLUDE\|<segundos>` | tapa la cámara virtual — **solo `--sim`** |

Figuras de `FORMATION`: **`linea`** (fila perpendicular al rumbo del líder),
**`cuna`** (V detrás del líder), **`circulo`** (anillo alrededor del líder).

---

## Comandos a los robots

Todos aceptan `<id>.` o `BROADCAST.`

### Movimiento directo

| comando | argumentos |
|---|---|
| `MOVE` | `<mm>` — positivo adelante, negativo atrás |
| `TURN` | `<grados>` |
| `WAIT` | `<ms>` |
| `RESET` | — reinicia el robot |

### Navegación a un punto

| comando | argumentos |
|---|---|
| `GT` | `<x>\|<y>[\|<segmento_mm>]` — navegación reactiva al punto |
| `ABORT_NAV` | — cancela la navegación en curso |
| `RANDOMW` | `[<segmento_mm>]` — caminata aleatoria |

> Hasta el 05-08 había además `GOTO`, `POSITIONGT` y `BUG2`. **No eran
> variantes**: los cuatro nombres caían en el mismo handler del firmware y el
> nombre ni llegaba adentro, así que daba exactamente igual cuál usaras. Se
> quedó `GT`.

### Enjambre

| comando | argumentos |
|---|---|
| `MEET` | `<x>\|<y>[\|<radio>]` — congregación sobre un punto, **sin líder**. Radio 150-600mm; vacío = lo calcula el firmware según cuántos robots hay |
| `DISPERSE` | `[<separación_mm>]` — separación objetivo, por defecto 600 |
| `CANCEL_CONGREGATION` | — corta congregación, formación o dispersión |
| `CONGREGATION` | `<slot>\|<x>\|<y>` — **crudo**; normalmente lo manda `BASE.CONGREGATION` |
| `FORMATION` | `<figura>\|<líder>\|<idx>\|<n>\|<eje>` — **crudo**, ídem |

### Calibración y control

| comando | argumentos |
|---|---|
| `GETPPR` | — consulta el PPR guardado |
| `SETPPR` | `<valor>\|TEMP` o `<valor>\|SAVE` |
| `PID` | `<kp>\|<ki>\|<kd>[\|SAVE]` |
| `KFPID` | `<q>\|<r>` — filtro de Kalman del PID |
| `SELFTEST` | `[<pwm%>]` — pulso a lazo abierto, por defecto 40% |
| `NAV_CONFIG` | `<clave>\|<valor>[\|SAVE]` |

Claves de `NAV_CONFIG`:
`YAW_SCALE` · `ARENA` (`|<w>|<h>`) · `SEGMENT_DIST` · `ARRIVAL_THRESHOLD` ·
`PARKING_DIST` · `WHEEL_DIST` · `GOAL_DEADBAND` · `REALIGN`

### Sensores

| comando | argumentos |
|---|---|
| `SENSOR_MASK` | `<L\|C\|R>\|<0\|1>` — **1 = ignorar** ese sensor |
| `SENSOR_THRESHOLD` | `<valor>` — umbral del IR central |
| `CLEAR_EVASION` | — limpia el estado de evasión |
| `RESET_EVASION` | — reinicia el contador de evasiones |
| `COLOR_READ` | `[<ganancia>\|<ms>]` — lectura RGBC del APDS9960; con argumentos fija la exposición (ganancia 1/4/16/64, integración 3-200 ms) |
| `COLOR_WB` | `[<R>\|<G>\|<B>]` o `RESET` — balance de blancos del sensor de color, propio de cada robot. Sin argumentos mide el patrón que tiene enfrente y lo guarda en NVS |
| `SEARCH_OBJECT` | — busca un objeto por color |

### Diagnóstico

| comando | argumentos |
|---|---|
| `GET_STATUS` | — estado completo: FSM, sensores, pose, yaw, IMU, EKF, calibración |
| `GET_YAW` | — solo el yaw de la IMU |
| `EKF_NAV` | `<0\|1>` — ⚠ **dejar en 0** (ver más abajo) |
| `CONFIG` | `SAVE\|<id>` — registra el robot en la Base |
| `SEND_COUNT_MESSAGE` | — contador de mensajes, para medir pérdida |

---

## Recetas

### Calibrar un robot

```
4.NAV_CONFIG|YAW_SCALE|1.0          resetear la escala
4.TURN|360                          ×3, anotar el error de cada uno
4.NAV_CONFIG|YAW_SCALE|<medido>|SAVE
4.SETPPR|574|TEMP                   valor de arranque
4.MOVE|500                          ×3, medir el recorrido real
4.SETPPR|<medido>|SAVE
```

O guiado: `BASE.CALIBRATE|4`

> El **PPR depende de la superficie** — absorbe el deslizamiento, no solo el
> encoder. Recalibrar al cambiar de piso, y no comparar valores entre sesiones
> sobre suelos distintos. Medido: el mismo robot dio 585.8 y 542.1 en pisos
> distintos, y ~2.8% de dispersión entre dos medidas seguidas. Si el PPR va a
> entrar en las métricas, usá tramos de 800-1000mm en vez de 500.

### Congregación (el escenario del paper)

```
BROADCAST.RANDOMW                   dispersar
BROADCAST.MEET|1200|850             congregar sobre un punto
```

Análisis: `python3 herramientas/analyze_logs.py --congregation`

### Dispersión

```
BROADCAST.DISPERSE|600
BROADCAST.CANCEL_CONGREGATION       para cortar
```

Los saltos se recortan a la arena **menos 350mm de inset**, o sea un área útil
de 1700×1050mm en la arena de 2400×1750. Con 4 robots, 600-700mm es cómodo;
**900mm o más no converge** y los robots quedan oscilando.

### Formación

```
BASE.FORMATION|linea|1
BASE.FORMATION|cuna|1|400
BASE.FORMATION|circulo|1|350
```

**Poné el líder cerca del centro.** Para `linea` y `cuna`, la Base valida que
*todos* los slots caigan dentro de la arena; si uno se sale te dice **cuál y por
cuántos mm**, y cuánto mover al líder. Con 3 seguidores y 300mm de espaciado la
fila necesita ±600mm libres a los costados; si la perpendicular no entra, cae
sola a la columna sobre el rumbo del líder y avisa.

**El `circulo` no se rechaza nunca.** Ahí el firmware usa `SafeRingSlotAngle`,
que agranda el radio hasta 2.5× y reparte los slots en el arco libre más largo
con su propio margen de pared. La Base solo avisa si el anillo nominal no entra
—porque el radio real puede terminar bastante mayor que el pedido— pero deja que
el robot resuelva.

Si a un seguidor no se le ve el marker, se le asigna **el slot que sobra** (no el
más cercano) y la Base lo avisa: puede cruzarse con los demás en el camino.

---

## Advertencias

**`EKF_NAV` va en 0.** El EKF corre pasivo y se mide contra el ArUco. Su error
mediano es bueno (24-89mm) pero diverge en episodios discretos de 6-20s, con p95
de hasta 2.7m. No confiarle la navegación hasta explicar esos episodios.
Criterio: p95 bajo 50mm para navegar, hasta ~150mm sirve solo para puentear
oclusiones cortas.

**Un comando mal escrito se descarta en silencio** *en el robot*. El firmware no
contesta si no reconoce el verbo. Ya pasó con un `CMD|MVE1.MOVE1.MOVE` que no
hizo nada y parecía un robot colgado.

Desde el 05-08 **la Base avisa antes de enviarlo**:

```
⚠ 'GOTO' no es un comando del firmware — se envía igual, pero el robot
  lo va a descartar sin avisar. BASE.HELP los lista.
```

Solo avisa, no bloquea. Aun así, si un robot "no obedece", buscá su fila `CMD|`
en el ConsoleLog antes de sospechar del hardware.

**`SENSOR_MASK` usa 1 para IGNORAR**, no para activar. Es al revés de lo que
parece.

---

## Dónde queda todo

| qué | dónde |
|---|---|
| Comandos enviados | `ConsoleLogs/Console_Log_*.csv`, filas `CMD|` |
| Poses y EKF | `PositionLogs/Position_Log_*.csv` |
| Ritmo del lazo | `Logs/Time_Log_*.csv` |
| Video | `Videos/Video_*.avi` |

La telemetría de alta frecuencia (`POSE`, `POSITION_RESPONSE`,
`LEADER_POSITION`, `NEIGHBOR_POSITIONS`) **no** se registra a propósito: taparía
el ConsoleLog.
