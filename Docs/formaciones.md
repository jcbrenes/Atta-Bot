# Formaciones

Cómo un grupo de robots diferenciales se acomoda en **línea**, **cuña** o
**círculo** alrededor de un líder que se mueve: qué decide la estación base, qué
difunde el líder y qué calcula cada seguidor por su cuenta.

Este documento describe la geometría tal como corre hoy en el firmware
(`Controller/AttaBot/navegacion.ino`) y en la base
(`Base/AttaBot_Base.py`), para poder reproducir el comportamiento o modificarlo
sin releer las dos implementaciones.

---

## Cómo se pide

Desde la consola de la base:

```
BASE.FORMATION|<figura>|<líder>[|<esp_mm>]
```

| | |
|---|---|
| `<figura>` | `linea` (fila perpendicular al rumbo del líder), `cuna` (V hacia atrás), `circulo` (anillo alrededor del líder) |
| `<líder>` | id del robot que define el marco de referencia; los demás lo siguen |
| `<esp_mm>` | espaciado, 300 mm por defecto |

Ejemplos:

```
BASE.FORMATION|linea|1|300      # fila de 300 mm de paso, líder Atta_1
BASE.FORMATION|cuna|1|400
BASE.FORMATION|circulo|2        # anillo de radio nominal 300 mm
```

La base no manda destinos: manda a cada seguidor su **índice de slot**, una sola
vez, con dos comandos del firmware:

```
NAV_CONFIG|PARKING_DIST|<esp_mm>
FORMATION|<figura>|<líderID>|<idx>|<n>|[eje°]
```

El líder recibe el mismo comando con índice 0 (lo ignora) para saber que tiene
que difundir su pose.

---

## El reparto de trabajo

Una formación se podría resolver entera en la base: ve a todos los robots por la
cámara cenital y podría calcular cada destino. No se hace así porque el líder
**se mueve**; si la base tuviera que recalcular y retransmitir cada destino con
cada centímetro que avanza, la formación viviría al ritmo del enlace inalámbrico
y se rompería con cada paquete perdido.

El trabajo se parte en tres, según qué información hace falta para cada decisión:

| decisión | quién | cuándo |
|---|---|---|
| qué slot le toca a cada robot | la base — es la única que los ve a todos, y sin vista global no se pueden evitar los cruces | una sola vez |
| dónde está el marco de referencia | el líder, por difusión — la formación se define respecto de **él**, no de la arena | continuo, 4 Hz |
| dónde queda mi slot y cómo llego | cada seguidor, local | con cada pose del líder |

```
   Base + cámara                Líder                    Seguidor i
   ve a todos                   define el marco          calcula dónde
   decide QUÉ slot              de referencia            S = L + k·s·â
        |                            |                        ^
        |  POSE 20 Hz ────────────►  |  LEADER_POSITION 4 Hz ─┘
        |                                                     ^
        |  FORMATION|figura|líder|idx|n|eje  (una sola vez) ───┘
        |  POSE 20 Hz ────────────────────────────────────────┘
```

Si la base se cae después de repartir los índices, la formación sigue viva: los
seguidores tienen todo lo que necesitan mientras el líder siga difundiendo.
Cortar la difusión del líder sí la rompe. Ese es el grado intermedio de
centralización de la plataforma, entre el comando central robot por robot y la
coordinación puramente local.

### Notación

| símbolo | qué es |
|---|---|
| `L`, `θL` | posición y rumbo del líder |
| `s` | espaciado pedido, en mm (`NAV_CONFIG\|PARKING_DIST`) |
| `n` | cantidad de seguidores |
| `i` | índice de slot del seguidor, `0 ≤ i < n` |
| `α` | giro opcional del eje de la fila |
| `ĥ`, `p̂` | versores del rumbo del líder y del eje de la fila |
| `â` | versor del brazo sobre el que se reparten los slots |
| `Sᵢ` | posición del slot `i` |
| `Wᵢ` | waypoint de aproximación (*staging*) del slot `i` |

---

## Etapa 1: la base reparte los slots

El problema no es dónde poner los robots, que lo fija la figura, sino **cuál va
en cuál**. Con tres seguidores hay seis asignaciones posibles y cinco de ellas
hacen que al menos dos robots se crucen de camino. Cruzarse en una arena chica
con navegación reactiva significa que se detectan mutuamente como obstáculo, se
evaden, y la formación tarda el doble o entra en bloqueo.

La solución es un **emparejamiento monótono**: se proyecta cada seguidor sobre el
eje de la fila, se ordenan los slots sobre ese mismo eje, y se emparejan por
rango — el que ya está más a la izquierda recibe el slot más a la izquierda.

```
p̂     = (cos(θL + 90° + α), sin(θL + 90° + α))     eje de la fila
κ(j)  = (Pj − L) · p̂                               clave del seguidor j
κs(i) = σ(i) · k(i)                                 clave del slot i
```

`k(i)` y `σ(i)` son el anillo y el lado que se definen en la etapa 3. Para el
círculo las claves son angulares: `κ(j) = atan2(yj − yL, xj − xL) mod 2π` y
`κs(i) = 2πi/n`. Ambas listas se ordenan de menor a mayor y se emparejan por
posición en el orden.

### Por qué el orden por rango no cruza

Sean `a₁ ≤ a₂ ≤ … ≤ aₙ` las coordenadas de los seguidores sobre el eje `p̂` y
`b₁ < b₂ < … < bₙ` las de los slots sobre ese mismo eje. Si cada seguidor `i` va
al slot `i` por un segmento recto, su coordenada proyectada en el instante
normalizado `t ∈ [0,1]` es la interpolación `pᵢ(t) = (1−t)·aᵢ + t·bᵢ`, y para un
par `i < j`:

```
pj(t) − pi(t) = (1−t)·(aj − ai) + t·(bj − bi)
```

Por construcción del emparejamiento por rango, `j > i` implica `aj ≥ ai` y
`bj > bi`, así que ambos sumandos son no negativos y el segundo es estrictamente
positivo para `t > 0`. Una combinación convexa de cantidades no negativas no
cambia de signo: la diferencia se mantiene positiva en todo el recorrido, y dos
puntos con distinta proyección sobre `p̂` son distintos. No hay cruce.

El recíproco explica por qué cualquier otra asignación falla: introduce al menos
una inversión (`ai < aj` pero `bi > bj`), ahí los dos sumandos tienen signos
opuestos, la diferencia es continua y cambia de signo, y por el teorema del valor
intermedio se anula en algún `t` interior. **Ese cero es el cruce.**

```
      por rango: 0 cruces              orden invertido: 2 cruces
      o───────────────□                o──────╲   ╱──────□
      o───────────────□                o───────╳─╳───────□
      o───────────────□                o──────╱   ╲──────□
   (o = seguidor, □ = slot sobre el eje de la fila)
```

### Qué no garantiza

La garantía es limpia pero angosta, y vale más saber dónde termina:

- **Es sobre la geometría planificada, no sobre la trayectoria ejecutada.** Los
  robots no van en línea recta: pasan por un waypoint de aproximación, avanzan a
  velocidades distintas y desvían por evasión reactiva. Dice que los *destinos*
  no obligan a cruzarse, no que el recorrido real no lo haga.
- **Es topológica, no métrica.** Conserva el orden, no la separación: dos robots
  pueden mantener el orden y aun así acercarse lo suficiente para dispararse la
  evasión por infrarrojo.
- **Supone el líder quieto durante el tránsito.** Si se mueve, los slots se
  mueven con él y las `bᵢ` dejan de ser constantes.
- **Los robots que la cámara no ve quedan fuera del argumento.** Reciben
  `κ = +∞` y el slot que sobra, de modo que su posición real no participa del
  orden, y la base lo advierte por consola. Devolver `0` en vez de `+∞` es peor
  que no ordenar: `0` es una coordenada lateral perfectamente válida, así que un
  robot invisible se cuela entre los visibles y les corre el slot a todos,
  degradando la asignación en silencio justo cuando más falta hace.
- **El círculo no hereda la garantía.** Sobre una circunferencia el orden es
  cíclico y el corte en 0° es arbitrario: el emparejamiento sin cruces en un
  ciclo exige elegir el desplazamiento rotacional correcto entre `n` posibles, y
  ordenar por rango con un corte fijo es sólo uno de esos `n`. Para el círculo la
  asignación es una heurística razonable, no un teorema.

### Validación contra la arena

Antes de transmitir nada, la base comprueba que todo slot caiga dentro de la
arena con un margen de 250 mm respecto de las paredes. Si una fila perpendicular
no cabe, se reintenta con el eje girado `α = 90°` — la fila se vuelve columna —
antes de rechazar. Si aun así no cabe, el mensaje de error indica qué slot falla
y por cuántos milímetros: un rechazo por 9 mm y uno por medio metro exigen
acciones distintas del operador.

El círculo es la excepción y no se valida acá, porque el firmware lo resuelve con
su propio algoritmo de anillo seguro y compararlo contra un anillo plano de radio
nominal sólo producía rechazos falsos.

> Las dimensiones de la arena se configuran con `NAV_CONFIG|ARENA`; no están
> tomadas del tamaño del cuadro de la cámara, que es otra cosa.

---

## Etapa 2: el líder difunde su pose

El líder recibe su propia pose de la base a unos 20 Hz y la reemite por difusión
(`LEADER_POSITION`), limitada a **4 Hz**. Reemitir las veinte saturaría el enlace
sin aportar nada, porque los seguidores no replanifican más rápido que eso. Al ir
por difusión, un seguidor más no cuesta un mensaje más.

Antes de difundir, la pose pasa por una media móvil de `N = 4` muestras. El
ángulo no se puede promediar directamente — entre 359° y 1° el promedio
aritmético da 180°, exactamente el lado contrario — así que se promedian el seno
y el coseno y se recompone:

```
x̄ = (1/N)·Σ xt          ȳ = (1/N)·Σ yt
θ̄ = atan2( Σ sin θt , Σ cos θt )
```

La ventana se vacía si la muestra nueva cae a más de 100 mm de la media vigente:
en ese caso el líder se movió de verdad, o la cámara lo leyó mal, y en cualquiera
de los dos casos promediar con lo viejo introduce un retardo que miente.

Del lado del seguidor hay una segunda defensa contra el ruido: el objetivo de
navegación sólo se reubica si el nuevo cae a más de **40 mm** del actual. Sin esa
banda muerta el destino se corre unos milímetros con cada mensaje y el navegador
termina re-apuntando contra el ruido de la localización visual en vez de contra
el movimiento real del líder.

---

## Etapa 3: el seguidor calcula su slot

Cada seguidor conoce cuatro cosas: la pose del líder que acaba de llegar, su
índice `i`, el total `n` y el espaciado `s`. Con eso arma la posición de su slot
sin hablar con nadie.

El índice se descompone en **anillo** y **lado**: los pares van a un lado, los
impares al otro, y cada par de índices sube un anillo.

```
k(i) = ⌊i/2⌋ + 1                 anillo (1, 1, 2, 2, 3, …)
σ(i) = +1 si i es par, −1 si impar
```

Con `ĥ = (cos θL, sin θL)` y `p̂` el eje de la fila, la dirección del brazo y la
posición del slot son:

```
        ⎧ σ(i)·p̂                    línea
â(i) =  ⎨
        ⎩ (σ(i)·p̂ − ĥ)/√2          cuña

Sᵢ   = L + k(i)·s·â(i)
```

```
      línea                    cuña                    círculo
        □ 2                        □ 2
        □ 0                   ╲                          □ 1
   ●───────►ĥ                   □ 0                    ●──────□ 0  ĥ
        □ 1                   ●───────►ĥ                  □ 2
                                □ 1  ╱                 (anillo seguro)
   â = σ·p̂                 â = (σ·p̂ − ĥ)/√2
```

La normalización de la cuña **no es cosmética**. Como `p̂` y `ĥ` son unitarios y
perpendiculares, `‖σ·p̂ − ĥ‖ = √2`; dividir por `√2` es lo que hace que `s`
signifique lo mismo en las dos figuras: distancia del líder al primer slot, y
separación entre slots consecutivos del mismo brazo. Sin ese `1/√2` el slot `k`
queda a `k·s·1,414` del líder — 566 mm cuando se piden 400. Que el brazo caiga
exactamente a 135° se sigue de la misma cuenta:

```
â·ĥ = (σ·(p̂·ĥ) − ĥ·ĥ)/√2 = (0 − 1)/√2 = −√2/2   ⟹   ∠(â, ĥ) = 135°
```

> **La fórmula del slot está escrita dos veces**: en el firmware, que la usa para
> navegar (`UpdateCongregationGoal` en `navegacion.ino`), y en la base, que la
> replica para validar contra los límites de la arena (`slotOffset` en
> `AttaBot_Base.py`). Cualquier cambio toca los dos archivos; tocar sólo el
> firmware deja a la base rechazando formaciones que en realidad caben.

### El anillo seguro del círculo

Un anillo de radio fijo alrededor de un líder pegado a la pared pone la mitad de
los slots fuera de la arena. En vez de rechazar la formación, el firmware busca
un anillo que sí entre (`SafeRingSlotAngle`): prueba radios crecientes
`g ∈ {1; 1,2; 1,4; 1,7; 2,0; 2,5}` sobre el radio nominal, muestrea cada uno en
72 puntos, y se queda con el arco contiguo más largo que respeta 200 mm de margen
a las paredes. Acepta ese radio si el arco alcanza para todos:

```
R · arco ≥ n · 250 mm            φᵢ = a₀ + (i + ½)·arco/n
```

Los 250 mm por robot son el espacio mínimo para que no se toquen. Si ningún radio
cumple, se usa el mayor y se aprieta: mejor una formación apretada que ninguna.

---

## Etapa 4: la entrada por aproximación

Conocer el slot no basta: la recta desde donde está el robot hasta su slot puede
pasar por encima del líder o cruzar el slot de un vecino. Por eso la navegación
es en dos tramos, y el punto intermedio depende de la figura:

```
       ⎧ Sᵢ − 150·ĥ                             línea y cuña
Wᵢ  =  ⎨
       ⎩ L + (R + 150)·(cos φᵢ, sin φᵢ)         círculo
```

En línea y cuña el waypoint queda 150 mm por detrás del slot, de modo que cada
robot entra por su propio carril, paralelo al de los demás. En círculo la
aproximación es radial: se entra desde afuera del anillo, nunca cruzándolo.

Al alcanzar `Wᵢ` el robot marca la etapa como cumplida y el objetivo pasa a ser
`Sᵢ`. De ahí en más sigue recalculando el slot con cada difusión del líder, así
que si el líder se mueve la formación lo acompaña.

El arranque de la flota se escalona con una espera de `200 ms × id` para que los
robots no salgan todos en el mismo instante.

---

## Secuencia completa

**Base, una sola vez:**

```
tomar pose del líder; si no es visible, abortar
F ← todos los robots menos el líder;  n ← |F|;  α ← 0
si figura ≠ círculo y algún slot se sale de la arena:
    si figura = línea y con α = 90° sí cabe:  α ← 90°      (la fila es columna)
    si no: informar qué slot falla y por cuántos mm; abortar
p̂ ← eje de la fila(θL, α)
para cada j en F:
    κ[j] ← +∞                        si j no es visible   (al final, NO al medio)
           atan2(yj−yL, xj−xL) mod 2π   si figura = círculo
           (Pj − L)·p̂                   en otro caso
F* ← F ordenado por κ;   S* ← [0…n−1] ordenado por κs
para r en 0…n−1:
    enviar a F*[r]:  NAV_CONFIG|PARKING_DIST|s
                     FORMATION|figura|líder|S*[r]|n|α
enviar al líder: FORMATION|figura|líder|0|n|α                (su índice se ignora)
```

**Firmware, en cada seguidor:**

```
al recibir FORMATION:
    guardar figura, i, n, α;  esLíder ← (líder = miID)
    slotFijado ← falso;  aproximaciónHecha ← falso
    encolar Esperar(200 ms × miID);  encolar PedirPosición

al recibir LEADER_POSITION (≈4 Hz):
    L, θL ← suavizar(x, y, θ)        media móvil circular, N = 4
    recalcularObjetivo()

recalcularObjetivo():
    ĥ ← (cos θL, sin θL);   p̂ ← eje de la fila(θL, α)
    si figura ∈ {línea, cuña}:
        k ← ⌊i/2⌋+1;   σ ← +1 si i par, −1 si impar
        â ← σ·p̂;   si figura = cuña: â ← (â − ĥ)/√2      (rayo a 135°, unitario)
        S ← L + k·s·â
        W ← S si aproximaciónHecha, si no S − 150·ĥ
    si no (círculo):
        si ¬slotFijado:
            rumbo ← atan2(yYo − yL, xYo − xL)            exige pose propia fresca
            φ, R ← ánguloDeAnilloSeguro(L, i, n, s, rumbo)
            slotFijado ← verdadero
        S ← L + R·(cos φ, sin φ)
        W ← L + (R + 150 si falta aproximación)·(cos φ, sin φ)
    si ‖W − objetivo‖ > 40 mm:  objetivo ← W             banda muerta vs. ruido

al llegar al objetivo:
    si ¬aproximaciónHecha: aproximaciónHecha ← verdadero; recalcularObjetivo()
    si no: quedarse, siguiendo al líder mientras siga difundiendo
```

**Anillo seguro (sólo círculo):**

```
para cada g en {1; 1,2; 1,4; 1,7; 2,0; 2,5}:
    R ← R₀·g
    marcar cuáles de 72 puntos del anillo caen a ≥ 200 mm de toda pared
    si los 72 son seguros: devolver reparto uniforme, radio R
    arco ← la racha circular contigua más larga de puntos seguros
    si R·arco ≥ n·250 mm  o  g es el último:
        devolver a₀ + (i + ½)·arco/n, radio R
```

---

## Precisión medida

Cuatro robots (un líder y tres seguidores) en una arena de 2,4 × 1,75 m, con
localización por ArUco cenital y odometría corregida por cámara. Distancia y
ángulo de cada seguidor respecto del líder al final de la corrida; el ángulo se
mide desde el rumbo del líder.

**Línea**, `s = 300 mm` — tres corridas:

| slot −300 | slot +300 | slot +600 |
|---|---|---|
| 319 mm / −87° | 311 mm / +86° | 605 mm / +91° |
| 316 mm / −86° | 294 mm / +89° | 769 mm / +108° |
| 313 mm / +91° | 297 mm / −92° | 593 mm / +91° |

La mejor corrida cierra con **7 mm** de error en el slot más lejano y menos de 2°
de desviación angular. El único fallo — el slot lejano de la segunda corrida —
fue de un robot cuyo marcador fiducial se perdía de vista con frecuencia, así que
navegaba sin realimentación visual: es un problema del marcador, no de la lógica
de slots.

**Cuña**, `s = 400 mm` — mediana de los últimos 8 s del registro de posiciones:

| | slot +135° | slot −135° | slot +135° (k = 2) |
|---|---|---|---|
| pedido | 400 mm | 400 mm | 800 mm |
| medido | 423 mm (+23) | 425 mm (+25) | 837 mm (+37) |

El residuo de 23–37 mm queda por debajo de la tolerancia de llegada de 50 mm, y
es del mismo orden que el de la línea. Los ángulos se mantuvieron entre ±131° y
±139°.

Un error de escala en la geometría del slot tiene una firma reconocible: **un
factor constante repartido por igual entre todos los slots**, con los ángulos
correctos. Si los tres seguidores quedan a `×1,41` de lo pedido, el sospechoso es
la normalización de la cuña, no la calibración de los robots.

---

## Límites conocidos

1. **El círculo no está validado en hardware.** Es la única de las tres figuras
   que no se ha ejecutado con robots reales, y además la única cuyo espaciado lo
   puede reescribir el anillo seguro.
2. **El espaciado del círculo no significa lo mismo que el de las otras dos
   figuras.** En línea y cuña `s` es exactamente la distancia al primer slot; en
   círculo es un radio *nominal* que el anillo seguro puede multiplicar hasta 2,5
   veces. Quien pida 400 mm en las tres figuras obtiene tres geometrías
   distintas, y hoy eso sólo se advierte por consola.
3. **La asignación anti-cruce es monótona, no óptima.** Minimiza cruces, que es
   lo que rompe la formación, pero no la distancia total recorrida. Con cuatro
   robots la diferencia no se nota; con diez podría.
4. **Ningún seguidor sabe de los demás.** Si dos se estorban en el camino, lo
   resuelve la evasión reactiva, no la formación. Es coherente con el diseño
   descentralizado, pero significa que el tiempo de convergencia depende de la
   configuración inicial.
