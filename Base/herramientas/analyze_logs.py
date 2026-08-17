#!/usr/bin/env python3
"""
analyze_logs.py — Métricas de navegación desde los logs de la base
====================================================================
Fase 1 del plan de métricas: parsea los CSV existentes (PositionLogs +
ConsoleLogs, formato lab — también los SIM_ generados desde Webots) y extrae
métricas cuantitativas por "run" de navegación:

  - convergence_time  : segundos desde el inicio del GT hasta 'NAV: llegó'
  - steps             : REQUEST_POSITION durante el run
  - path_length       : Σ|linearDisplacement| del PositionLog en la ventana
  - path_efficiency   : distancia_recta / path_length (1.0 = trayecto perfecto)
  - final_error       : distancia de la pose final al goal (si el goal se conoce
                        por el ack 'GT iniciado: goal=(x,y)')
  - evasions          : evasiones completadas ('Cooldown de evasión completado')
  - obstacle_events   : mensajes CHECK_OBSTACLE en la ventana

Un run empieza con el ack 'GT iniciado' (goal conocido) o, si no hay ack (debug
apagado), con el primer REQUEST_POSITION tras ≥8s sin solicitudes; termina con
'NAV: llegó a (x,y)'. Los runs de congregación se etiquetan con el mensaje
'CONGREGATION: slot' si aparece en la ventana.

Uso:
    python analyze_logs.py                    # sesión más reciente
    python analyze_logs.py --session 03-07_10-04
    python analyze_logs.py --all              # resumen de todas las sesiones
    python analyze_logs.py --all --csv runs.csv

Modo congregación (experimento del paper: N robots se congregan bajo distintas
topologías de obstáculos). Reutiliza el mismo binning anti-jitter y saca del
PositionLog el centroide, el radio del grupo (spread), la distancia mínima
inter-robot, el tiempo de convergencia (radio R) y el recorrido por robot:
    python analyze_logs.py --congregation --session SIM_07-07_23-09
    python analyze_logs.py --congregation --session <run> --layout 3_cajas --plot
    python analyze_logs.py --congregation --all --csv congregacion.csv
    python analyze_logs.py --congregation --radius 400 --hold 3   # ajustar R
"""

import argparse
import csv
import glob
import itertools
import json
import math
import os
import re

# Los scripts viven en un subdirectorio, asi que la carpeta Base — donde estan
# los logs, los paquetes de datos y configSystem.json — es la de arriba.
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
POS_DIR = os.path.join(BASE_DIR, 'PositionLogs')
CON_DIR = os.path.join(BASE_DIR, 'ConsoleLogs')

RE_GT_START = re.compile(r'GT iniciado: goal=\((-?[\d.]+),(-?[\d.]+)\)')
RE_ARRIVED = re.compile(r'NAV: llegó a \((-?[\d.]+),(-?[\d.]+)\)')
IDLE_GAP_S = 8.0     # sin REQUEST_POSITION por este tiempo = run nuevo (fallback)

# Comandos de la Base que cierran el random walk y abren la fase de agrupación.
# Aparecen en el ConsoleLog como filas `CMD|<comando>|...` (ver
# Base.logCommand); en logs anteriores a 2026-08-05 no existen y el arranque se
# infiere. MEET congrega sobre un punto, CONGREGATION sobre un líder y FORMATION
# sobre una figura, pero para separar fases los tres marcan el mismo borde.
MEET_CMDS = frozenset(('MEET', 'CONGREGATION', 'FORMATION'))

# Calibración IR: bitmap del firmware = 4·IZQ + 2·CEN + 1·DER (IZQ/DER = IR de
# pin33/pin27; CEN = proximidad APDS9960). Máscara: SENSOR_MASK|L|C|R|0/1.
IR_BITS = ((4, 'IZQ'), (2, 'CEN'), (1, 'DER'))
IR_MASK = {'IZQ': 'L', 'CEN': 'C', 'DER': 'R'}
IR_GHOST_DIST = 350.0   # mm — disparo a más de esto de todo obstáculo = fantasma

# Diámetro del AttaBot: unidad de normalización de las métricas del experimento
# de topología (el radio del cuerpo que dibuja la Base es 75mm). Todas las
# distancias del paper se reportan en múltiplos de d para que los resultados
# sean comparables entre arenas y entre lab/simulación.
ATTA_DIAMETER_MM = 105.0


def _median(vals):
    vals = sorted(vals)
    n = len(vals)
    return vals[n // 2] if n % 2 else (vals[n // 2 - 1] + vals[n // 2]) / 2


def session_tag(path):
    """'Position_Log_03-07_10-04_Robots_2.csv' → '03-07_10-04_Robots_2'
    (conserva el prefijo SIM_ si existe)."""
    name = os.path.basename(path)
    return re.sub(r'^(Position|Console)_Log_', '', name).rsplit('.', 1)[0]


def find_sessions(extra_dir=None):
    """Retorna [(tag, position_csv, console_csv|None)] ordenado por mtime.

    `extra_dir` agrega otro par PositionLogs/ConsoleLogs a la búsqueda. Lo usa
    `run_campaign` con la carpeta del manifiesto, para que una campaña
    empaquetada aparte (con sus propios logs al lado) se analice sin tener que
    devolver los CSV a Base/.

    Acepta CUALQUIER csv en PositionLogs, no solo 'Position_Log_*': en el lab las
    corridas se renombran al escenario (SinObs_ALin.csv) apenas terminan, y con
    el glob viejo esas quedaban invisibles para todas las herramientas.

    El emparejamiento con la consola es por nombre y, si falla, por mtime (±90s):
    al renombrar a mano es fácil renombrar el PositionLog y olvidar el Console_Log,
    y ambos archivos se cierran juntos al final de la corrida.
    """
    pos_dirs, con_dirs = [POS_DIR], [CON_DIR]
    if extra_dir:
        pos_dirs.append(os.path.join(extra_dir, 'PositionLogs'))
        con_dirs.append(os.path.join(extra_dir, 'ConsoleLogs'))
    positions = sorted((p for d in pos_dirs
                        for p in glob.glob(os.path.join(d, '*.csv'))),
                       key=os.path.getmtime)
    consoles = [c for d in con_dirs for c in glob.glob(os.path.join(d, '*.csv'))]
    by_tag = {session_tag(p): p for p in consoles}

    out = []
    for p in positions:
        tag = session_tag(p)
        con = by_tag.get(tag)
        if con is None and consoles:
            near = min(consoles, key=lambda c: abs(os.path.getmtime(c)
                                                   - os.path.getmtime(p)))
            if abs(os.path.getmtime(near) - os.path.getmtime(p)) <= 90:
                con = near
        out.append((tag, p, con))
    return out


def load_console(path):
    """[(t, robot_id, message)] — tolera filas corruptas."""
    events = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                events.append((float(row['time']), row['idrobot'], row['message']))
            except (ValueError, KeyError, TypeError):
                continue
    return events


def load_positions(path):
    """{robot_id: [(t, x, y, angle, lin_disp)]} — tolera filas corruptas."""
    tracks = {}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                tracks.setdefault(row['idrobot'], []).append(
                    (float(row['time']), float(row['x']), float(row['y']),
                     float(row['angle']), float(row['linearDisplacement'])))
            except (ValueError, KeyError, TypeError):
                continue
    return tracks


def segment_runs(events, robot_id):
    """Corta los eventos de UN robot en runs de navegación.

    Retorna [{'t0', 't1', 'goal': (x,y)|None, 'steps', 'evasions',
              'obstacle_events', 'kind': 'GT'|'CONGREGATION', 'arrived': bool}]
    """
    runs = []
    current = None
    last_request = None

    def close(t1, arrived):
        nonlocal current
        if current is not None:
            current['t1'] = t1
            current['arrived'] = arrived
            runs.append(current)
            current = None

    for t, rid, msg in events:
        if rid != robot_id:
            continue

        m = RE_GT_START.search(msg)
        if m:
            close(t, False)   # run anterior sin llegada = interrumpido
            current = {'t0': t, 'goal': (float(m.group(1)), float(m.group(2))),
                       'steps': 0, 'evasions': 0, 'obstacle_events': 0,
                       'kind': 'GT'}
            continue

        if msg.startswith('REQUEST_POSITION'):
            if current is None:
                # Sin ack de inicio (debug apagado): el primer REQUEST tras un
                # silencio largo abre un run sin goal conocido
                if last_request is None or t - last_request >= IDLE_GAP_S:
                    current = {'t0': t, 'goal': None, 'steps': 0, 'evasions': 0,
                               'obstacle_events': 0, 'kind': 'GT'}
            if current is not None:
                current['steps'] += 1
            last_request = t
            continue

        if current is None:
            continue
        if 'CONGREGATION: slot' in msg:
            current['kind'] = 'CONGREGATION'
        elif 'Cooldown de evasión completado' in msg:
            current['evasions'] += 1
        elif msg.startswith('CHECK_OBSTACLE'):
            current['obstacle_events'] += 1
        elif RE_ARRIVED.search(msg):
            close(t, True)

    return runs


def binned_path(window, bin_s=1.0):
    """Camino recorrido sobre medianas de posición por ventana de 1s.

    Sumar |linearDisplacement| crudo infla el camino ~10-15x: el jitter ArUco
    (σ≈10-30mm) dispara filas de ≥4mm a ~20Hz aunque el robot esté quieto.
    La mediana por bin promedia el ruido de media cero y deja el movimiento
    real (≤180mm/s) intacto.
    """
    bins = {}
    for t, x, y, *_ in window:
        bins.setdefault(int(t / bin_s), []).append((x, y))

    centers = [(_median([p[0] for p in pts]), _median([p[1] for p in pts]))
               for _, pts in sorted(bins.items())]
    path = sum(math.dist(centers[i], centers[i + 1])
               for i in range(len(centers) - 1))
    return path, centers


def run_metrics(run, track):
    """Completa el run con las métricas del PositionLog en [t0, t1]."""
    window = [p for p in track if run['t0'] <= p[0] <= run['t1']]
    if len(window) < 2:
        return None
    path, centers = binned_path(window)
    if len(centers) < 2:
        return None
    x0, y0 = centers[0]
    x1, y1 = centers[-1]
    straight = math.dist((x0, y0), (x1, y1))
    metrics = {
        'duration_s': round(run['t1'] - run['t0'], 1),
        'path_mm': round(path, 0),
        'straight_mm': round(straight, 0),
        'efficiency': round(straight / path, 2) if path > 0 else 0.0,
        'final_error_mm': (round(math.dist((x1, y1), run['goal']), 0)
                           if run['goal'] else None),
    }
    metrics.update(run)
    return metrics


# ─────────────────────────────────────────────────────────────────────────
# MÉTRICAS DE CONGREGACIÓN GRUPAL
# El experimento del paper: N robots se congregan bajo distintas topologías de
# obstáculos. Estas métricas salen de los mismos PositionLogs/ConsoleLogs y
# reutilizan el binning de 1s anti-jitter de binned_path().
# ─────────────────────────────────────────────────────────────────────────

def real_tracks(tracks):
    """Descarta el marcador de origen (id 0) y no-reconocidos (id -1); deja los
    robots (id ≥ 1) como {rid: track}."""
    out = {}
    for rid, track in tracks.items():
        try:
            if int(rid) >= 1:
                out[rid] = track
        except (ValueError, TypeError):
            continue
    return out


def binned_track(track, bin_s=1.0):
    """{k: (x_med, y_med)} — mediana de posición por bin de bin_s s (anti-jitter,
    misma idea que binned_path pero conservando el índice de bin para alinear
    robots entre sí)."""
    bins = {}
    for t, x, y, *_ in track:
        bins.setdefault(int(t / bin_s), []).append((x, y))
    return {k: (_median([p[0] for p in pts]), _median([p[1] for p in pts]))
            for k, pts in bins.items()}


def moving_bins(tracks, bin_s=1.0, move_mm=40.0):
    """{rid: {k: bool}} — si el robot se desplazó entre el bin k−1 y el k.

    Se mide sobre la mediana por bin, no sobre linearDisplacement crudo: con
    σ≈30mm de ruido ArUco un robot QUIETO reporta desplazamientos de ≥4mm a
    20Hz, así que ninguna fase tendría borde. La mediana de ~20 muestras baja
    el ruido a ~8mm y deja el umbral de 40mm holgado contra los ~180mm/s reales.
    """
    out = {}
    for rid, track in tracks.items():
        b = binned_track(track, bin_s)
        flags = {}
        for k in sorted(b):
            prev = b.get(k - 1)
            flags[k] = prev is not None and math.dist(b[k], prev) >= move_mm
        out[rid] = flags
    return out


def _blocks(active, bins, gap_s=8.0, bin_s=1.0):
    """[(t_ini, t_fin)] — bloques de actividad contiguos tolerando pausas de
    hasta `gap_s`.

    La tolerancia hace falta porque un robot en random walk alterna avance y
    GIRO EN EL MISMO SITIO, y el giro no desplaza el centro: sin ella cada giro
    partiría la fase en dos. Y separar por bloques (en vez de tomar el primer o
    el último bin con movimiento) evita que un blip suelto —el operador
    acomodando un robot antes de arrancar— defina el borde de una fase.
    """
    out = []
    for k in bins:
        if not active.get(k):
            continue
        if out and (k - out[-1][1]) * bin_s <= gap_s:
            out[-1][1] = k
        else:
            out.append([k, k])
    return [(a * bin_s, b * bin_s) for a, b in out]


def detect_phases(tracks, events, bin_s=1.0, move_mm=40.0, gap_s=8.0,
                  min_block_s=10.0):
    """Separa la corrida en RANDOM WALK → tiempo muerto → MEET (congregación).

    El protocolo del lab es: la base manda RANDOMW, los robots caminan, se
    quedan quietos, y el operador manda el MEET A MANO.

      - `meet_start`: si el ConsoleLog trae el comando de la Base (filas `CMD|`,
        logs desde 2026-08-05) se usa ese instante y la fase se SABE. Si no, se
        infiere con el primer REQUEST_POSITION de cualquier robot: durante el
        random walk el robot nunca pide su pose y durante el MEET la pide
        continuamente, así que el borde es limpio (verificado en las 15 corridas
        del 30-07). Sin ConsoleLog se cae al valle de quietud más largo, que es
        el mismo instante pero con ±1 bin de incertidumbre.
        El dataset del paper es anterior al logueo de comandos, así que todo lo
        ya medido salió y sigue saliendo por el camino inferido.
      - `rw_end`: último bin con movimiento ANTES de meet_start.
      - `dead_s`: rw_end → meet_start. Es el tiempo del OPERADOR, no del
        experimento: incluirlo en la duración de la corrida (que es lo que da
        el largo del log o del video) la infla 20-60s.
      - `rw_start`: primer bin con movimiento. No es 0: la base ya está
        grabando cuando se manda el RANDOMW.

    Retorna None si no se puede ubicar el arranque del MEET.
    """
    flags = moving_bins(tracks, bin_s, move_mm)
    if not flags:
        return None
    all_bins = sorted({k for f in flags.values() for k in f})
    if not all_bins:
        return None
    active = {k: any(f.get(k) for f in flags.values()) for k in all_bins}

    meet_start, source = None, None
    # Preferido: el comando que mandó la Base, si quedó logueado.
    for t, _rid, msg in events:
        parts = msg.strip().split('|')
        if parts[0] == 'CMD' and len(parts) > 1 and parts[1] in MEET_CMDS:
            meet_start, source = t, f'CMD|{parts[1]}'
            break
    if meet_start is None:
        for t, _rid, msg in events:
            if msg.strip() == 'REQUEST_POSITION':
                meet_start, source = t, 'REQUEST_POSITION'
                break
    if meet_start is None:
        # Fallback sin consola: el hueco de quietud más largo que no sea el
        # arranque ni el final del log.
        best, gap0 = 0, None
        run0 = None
        for k in all_bins:
            if not active[k]:
                run0 = k if run0 is None else run0
            elif run0 is not None:
                if k - run0 > best and run0 > all_bins[0]:
                    best, gap0 = k - run0, k
                run0 = None
        if gap0 is None:
            return None
        meet_start, source = gap0 * bin_s, 'valle de quietud'

    blocks = _blocks(active, all_bins, gap_s, bin_s)
    if not blocks:
        return None
    # El random walk es el último bloque LARGO anterior al MEET. Exigir
    # `min_block_s` descarta los blips de manipulación: sin eso, un solo bin de
    # movimiento a 30s del comando se tomaba como toda la caminata (rw_s=0.0).
    # Un bloque que CRUZA el comando es la congregación, no la caminata: se
    # recorta en el comando y así solo compite por ser RW con su parte previa,
    # que en las corridas del 30-07 son décimas de segundo.
    prev = [(a, min(b, meet_start)) for a, b in blocks if a < meet_start]
    long_prev = [b for b in prev if b[1] - b[0] >= min_block_s]
    rw_start, rw_end = (long_prev or prev or [(meet_start, meet_start)])[-1]
    # La congregación llega hasta que el enjambre deja de moverse. Se toma el
    # ÚLTIMO bloque largo posterior al comando, no el primero: una congregación
    # puede pausarse (robot esperando pose, evasión) y retomar, y quedarse con
    # el primer bloque decía '32s' en corridas donde los robots seguían dando
    # vueltas a los 290s. Los bloques cortos del final sí se ignoran: son jitter
    # de marcador, no movimiento.
    post = [b for b in blocks if b[1] >= meet_start]
    long_post = [b for b in post if b[1] - b[0] >= min_block_s]
    last_move = (long_post or post or [(meet_start, meet_start)])[-1][1]

    return {
        'source': source,
        'rw_start_s': round(rw_start, 1),
        'rw_end_s': round(rw_end, 1),
        'rw_s': round(rw_end - rw_start, 1),
        'meet_start_s': round(meet_start, 1),
        'dead_s': round(meet_start - rw_end, 1),
        'last_move_s': round(last_move, 1),
        # Duración del MEET medida hasta que se detiene el último robot. Es el
        # techo: el criterio del 90% (t_conv_frac) suele cortar antes.
        'meet_s': round(last_move - meet_start, 1),
        'log_s': round(all_bins[-1] * bin_s, 1),
    }


def per_robot_phases(tracks, phases, bin_s=1.0, move_mm=40.0):
    """Por robot: cuánto tiempo estuvo EN MOVIMIENTO en cada fase y cuándo se
    detuvo definitivamente durante el MEET.

    `t_stop_s` es el tiempo de congregación de ESE robot (relativo al comando
    MEET): el instante de su último desplazamiento. Un robot que ya nace en su
    slot da 0.0 y uno que nunca se asienta da la duración completa.
    """
    flags = moving_bins(tracks, bin_s, move_mm)
    m0 = phases['meet_start_s']
    out = {}
    for rid, f in flags.items():
        rw = [k for k, mv in f.items()
              if mv and phases['rw_start_s'] <= k * bin_s < m0]
        mt = [k for k, mv in f.items() if mv and k * bin_s >= m0]
        out[rid] = {
            'rw_active_s': round(len(rw) * bin_s, 1),
            'meet_active_s': round(len(mt) * bin_s, 1),
            't_stop_s': round(max(mt) * bin_s - m0, 1) if mt else 0.0,
        }
    return out


def detect_leader(events):
    """Devuelve el idrobot del líder si algún mensaje declara 'soy líder'."""
    for _t, rid, msg in events:
        low = msg.lower()
        if 'soy líder' in low or 'soy lider' in low:
            return rid
    return None


def group_series(tracks, bin_s=1.0):
    """Serie temporal grupal sobre los bins comunes a TODOS los robots.

    Retorna [{'t', 'centroid', 'pos', 'spread', 'min_pair', 'compaction',
              'compaction_d', 'rms_d'}].
      - centroid  : media de las posiciones (centroide del enjambre)
      - spread    : max_i ‖p_i − centroid‖ (radio del grupo)
      - min_pair  : distancia mínima entre pares (proximidad de colisión)
      - compaction: √(Σ_i ‖p_i − centroide‖²) — la compactación del enjambre
        definida para el paper. Es una medida GRUPAL: penaliza a cualquier robot
        rezagado, a diferencia de 'spread' que solo mira al peor.
      - compaction_d : la anterior en diámetros de Atta (comparable entre arenas)
      - rms_d     : √(Σd²/N) en diámetros — la versión por-robot, que sí es
        comparable entre corridas con DISTINTO número de robots (compaction
        crece con √N aunque el enjambre esté igual de apretado).
    """
    binned = {rid: binned_track(t, bin_s) for rid, t in tracks.items()}
    binned = {rid: b for rid, b in binned.items() if b}
    if len(binned) < 2:
        return []
    common = sorted(set.intersection(*[set(b) for b in binned.values()]))
    series = []
    for k in common:
        pos = {rid: binned[rid][k] for rid in binned}
        pts = list(pos.values())
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        dists = [math.dist(p, (cx, cy)) for p in pts]
        spread = max(dists)
        sum_sq = sum(d * d for d in dists)
        compaction = math.sqrt(sum_sq)
        pairs = [math.dist(a, b) for a, b in itertools.combinations(pts, 2)]
        series.append({'t': k * bin_s, 'centroid': (cx, cy), 'pos': pos,
                       'spread': spread, 'min_pair': min(pairs),
                       'compaction': compaction,
                       'compaction_d': compaction / ATTA_DIAMETER_MM,
                       'rms_d': math.sqrt(sum_sq / len(pts)) / ATTA_DIAMETER_MM})
    return series


def convergence_time(series, radius, hold_s=3.0, t0=None):
    """Primer instante ≥ t0 con spread ≤ radius sostenido ≥ hold_s (o hasta el
    final del log). Retorna (t_abs, t_rel_a_t0, converged).

    Solo cuenta la convergencia que ocurre DESPUÉS del inicio del episodio (t0 =
    comando de congregación): si los robots ya arrancan agrupados, eso no es
    'haber convergido'."""
    if not series:
        return None, None, False
    if t0 is None:
        t0 = series[0]['t']
    after = [s for s in series if s['t'] >= t0]
    for i, s in enumerate(after):
        if s['spread'] > radius:
            continue
        # se mantiene ≤ radius durante los próximos hold_s segundos
        if all(o['spread'] <= radius for o in after[i:]
               if o['t'] <= s['t'] + hold_s):
            return s['t'], round(s['t'] - t0, 1), True
    return None, None, False


def convergence_time_frac(series, radius, frac=0.9, hold_s=3.0, t0=None,
                          target=None):
    """Primer instante ≥ t0 con al menos `frac` de los robots dentro de `radius`
    del centroide —o de `target`, si el experimento tiene un punto fijo—,
    sostenido ≥ hold_s. Retorna (t_abs, t_rel, converged).

    **Con destino fijo hay que pasar `target`.** Medido contra el centroide, un
    enjambre que se ATASCÓ antes de la barrera cuenta como convergido, porque
    quedó apretado alrededor de su propio centroide: en la campaña de sim del
    31-07 los escenarios de 2 diámetros daban 27s de "convergencia" con el
    centroide a 865-1664mm del punto y solo 2 de 10 robots del otro lado.

    Criterio del paper para enjambres grandes. `convergence_time` exige que
    TODOS entren (spread ≤ R), así que un único rezagado define el tiempo de la
    corrida entera — con 10 robots eso mide al peor, no al enjambre. Ojo: con 4
    robots 90% ≡ ⌈3.6⌉ ≡ 4, o sea idéntico al criterio estricto; los dos recién
    se separan a partir de ~10 robots.

    El centroide es instantáneo (definición de JC). Tiene el efecto de que un
    rezagado corre la referencia hacia sí mismo; es conservador, porque acerca
    el centroide al que falta y aleja a los que ya llegaron.
    """
    if not series:
        return None, None, False
    if t0 is None:
        t0 = series[0]['t']
    after = [s for s in series if s['t'] >= t0]

    def enough(s):
        pts = list(s['pos'].values())
        need = math.ceil(frac * len(pts))
        ref = target if target is not None else s['centroid']
        inside = sum(1 for p in pts if math.dist(p, ref) <= radius)
        return inside >= need

    # Si el criterio YA se cumple al inicio del episodio, no hubo congregación
    # que medir: el enjambre arrancó agrupado. Devolverlo como 0.0s metería
    # ceros en los boxplots del paper. Pasa sistemáticamente con N=2, donde el
    # centroide es el punto medio y "dentro del radio" ≡ estar a media distancia
    # del otro robot.
    if after and enough(after[0]):
        return None, None, False

    for i, s in enumerate(after):
        if not enough(s):
            continue
        if all(enough(o) for o in after[i:] if o['t'] <= s['t'] + hold_s):
            return s['t'], round(s['t'] - t0, 1), True
    return None, None, False


def per_robot_congregation(tracks, events, bin_s=1.0, t0=None):
    """Por robot: recorrido, ratio de ruta, pose final y evasiones.

    `t0` acota la medición al EPISODIO de congregación. Sin él, un GT previo en
    la misma sesión se sumaba al recorrido y al conteo de evasiones, inflando el
    ratio de ruta del run (medido 2026-07-27: 8.64 sobre el log completo contra
    7.9 sobre el episodio). Para la campaña del paper esto no es cosmético.
    """
    out = {}
    for rid, track in tracks.items():
        if t0 is not None:
            track = [s for s in track if s[0] >= t0]
            if len(track) < 2:
                continue
        path, centers = binned_path(track, bin_s)
        if not centers:
            continue
        straight = math.dist(centers[0], centers[-1]) if len(centers) >= 2 else 0.0
        # eventos de evasión IR: disparos de evasión ('MOVE interrumpido por IR —
        # evasión ...', 'Evasión ...'), excluyendo el 'Cooldown ... completado'
        # (el fin de la maniobra). Es la medida de cuánto forzó evadir el campo.
        evas = sum(1 for _t, r, m in events
                   if r == rid and 'evasi' in m.lower()
                   and 'cooldown' not in m.lower()
                   and (t0 is None or _t >= t0))
        out[rid] = {
            'path_mm': round(path),
            'straight_mm': round(straight),
            'efficiency': round(straight / path, 2) if path > 0 else 0.0,
            # Métrica de ruta del paper: recorrido ÷ distancia euclidiana directa.
            # 1.0 = fue en línea recta; 3.0 = dio tres veces la vuelta necesaria.
            # Es la inversa de 'efficiency', que se mantiene por compatibilidad.
            # None si el robot casi no se desplazó (típicamente el LÍDER, que se
            # queda quieto): ahí el cociente diverge —se midió 56.2 con 17mm de
            # recta— y ensuciaría los boxplots. El umbral es un diámetro de Atta.
            'route_ratio': (round(path / straight, 2)
                            if straight >= ATTA_DIAMETER_MM else None),
            'path_d': round(path / ATTA_DIAMETER_MM, 1),
            'final': (round(centers[-1][0], 1), round(centers[-1][1], 1)),
            'evasions': evas,
        }
    return out


def congregation_metrics(tag, pos_path, con_path, radius=450.0, hold_s=3.0,
                         t0=None, frac=0.9, target=None):
    """Agrega todas las métricas grupales de una sesión en un dict, o None si no
    hay ≥2 robots con trayectoria."""
    tracks = real_tracks(load_positions(pos_path))
    if len(tracks) < 2:
        return None
    events = load_console(con_path) if con_path else []
    series = group_series(tracks)
    if not series:
        return None
    # inicio del episodio: primer 'soy líder'/FORMATION/CONGREGATION, si existe
    if t0 is None:
        for _t, _rid, msg in events:
            low = msg.lower()
            if 'soy líder' in low or 'formación' in low or 'congregation' in low:
                t0 = _t
                break
    # Sin esos mensajes (debug del seguidor apagado, que es el caso de todo el
    # dataset del 30-07) el episodio arrancaría en 0 y el random walk entero
    # entraría en el tiempo de congregación y en el recorrido de cada robot.
    phases = detect_phases(tracks, events)
    if t0 is None and phases:
        t0 = phases['meet_start_s']
    t_abs, t_rel, conv = convergence_time(series, radius, hold_s, t0)
    tf_abs, tf_rel, conv_f = convergence_time_frac(series, radius, frac, hold_s,
                                                   t0, target)
    final = series[-1]
    # Con destino fijo, "llegaron" es su propia métrica y no se deduce del
    # spread: dice cuántos cruzaron, que es lo que compara las topologías.
    arrived = dist_target = None
    if target is not None:
        arrived = sum(1 for p in final['pos'].values()
                      if math.dist(p, target) <= radius)
        dist_target = round(math.dist(final['centroid'], target))
    return {
        'session': tag,
        'n_robots': len(tracks),
        'leader': detect_leader(events),
        'radius_mm': radius,
        'converged': conv,
        't_conv_abs_s': round(t_abs, 1) if t_abs is not None else None,
        't_conv_s': t_rel,
        # Criterio del paper: 'frac' de los robots dentro del radio
        'frac': frac,
        'converged_frac': conv_f,
        't_conv_frac_s': tf_rel,
        'final_compaction_d': round(final['compaction_d'], 2),
        'final_rms_d': round(final['rms_d'], 2),
        'min_compaction_d': round(min(s['compaction_d'] for s in series), 2),
        'final_spread_mm': round(final['spread']),
        'final_min_pair_mm': round(final['min_pair']),
        'min_pair_ever_mm': round(min(s['min_pair'] for s in series)),
        'final_centroid': (round(final['centroid'][0], 1),
                           round(final['centroid'][1], 1)),
        't0_s': round(t0, 1) if t0 is not None else None,
        'target': target,
        'arrived': arrived,
        'centroid_to_target_mm': dist_target,
        'phases': phases,
        'robot_phases': per_robot_phases(tracks, phases) if phases else {},
        'per_robot': per_robot_congregation(tracks, events, t0=t0),
        'series': series,
    }


def print_congregation(m, layout=None):
    """Imprime el reporte grupal de una sesión."""
    print(f"\n=== CONGREGACIÓN {m['session']}"
          f"{'  [topología: ' + layout + ']' if layout else ''} ===")
    lead = f" (líder Atta_{m['leader']})" if m['leader'] else ''
    print(f"robots: {m['n_robots']}{lead}   radio de convergencia R={m['radius_mm']:.0f}mm")
    ph = m.get('phases')
    if ph:
        print(f"FASES (inicio del MEET por {ph['source']}):")
        print(f"  random walk   {ph['rw_start_s']:>6.1f} → {ph['rw_end_s']:>6.1f} s"
              f"   ({ph['rw_s']:.1f} s)")
        print(f"  tiempo muerto {ph['rw_end_s']:>6.1f} → {ph['meet_start_s']:>6.1f} s"
              f"   ({ph['dead_s']:.1f} s — operador, no cuenta)")
        print(f"  congregación  {ph['meet_start_s']:>6.1f} → {ph['last_move_s']:>6.1f} s"
              f"   ({ph['meet_s']:.1f} s hasta que se detiene el último)")
        print(f"  log completo  {ph['log_s']:.1f} s")
    if m['converged']:
        print(f"CONVERGIÓ en {m['t_conv_s']:.1f} s "
              f"(t absoluto {m['t_conv_abs_s']:.1f} s)")
    else:
        print(f"NO convergió a R={m['radius_mm']:.0f}mm "
              f"(spread final {m['final_spread_mm']} mm)")
    pct = int(m['frac'] * 100)
    if m['converged_frac']:
        print(f"{pct}% dentro de R en          : {m['t_conv_frac_s']:.1f} s")
    else:
        print(f"NO llegó a tener {pct}% dentro de R")
    print(f"spread final (radio del grupo) : {m['final_spread_mm']} mm")
    print(f"compactación final             : {m['final_compaction_d']:.2f} d "
          f"(√Σd² al centroide; mejor de la corrida {m['min_compaction_d']:.2f} d)")
    print(f"  ídem por robot (RMS)         : {m['final_rms_d']:.2f} d")
    print(f"centroide final                : {m['final_centroid']} mm")
    print(f"dist. mín. inter-robot (final) : {m['final_min_pair_mm']} mm"
          f"   (mínimo histórico {m['min_pair_ever_mm']} mm)")
    print(f"{'robot':>7} {'recorrido':>10} {'recta':>7} {'ratio':>6} "
          f"{'evas':>4} {'RW act':>7} {'MEET act':>9} {'se detuvo':>10}  pose_final")
    rp = m.get('robot_phases', {})
    for rid in sorted(m['per_robot'], key=lambda r: int(r)):
        p = m['per_robot'][rid]
        f = rp.get(rid, {})
        ratio = f"{p['route_ratio']:.2f}" if p['route_ratio'] is not None else '  —'
        print(f"  Atta_{rid:<2} {p['path_mm']:>9.0f} {p['straight_mm']:>7.0f} "
              f"{ratio:>6} {p['evasions']:>4} "
              f"{f.get('rw_active_s', 0):>6.0f}s {f.get('meet_active_s', 0):>8.0f}s "
              f"{f.get('t_stop_s', 0):>9.0f}s  {p['final']}")


# ── Estadística de la campaña ────────────────────────────────────────────────
# ANOVA de una vía sin scipy (no está instalado). La beta incompleta regularizada
# da el p-valor de F; es el algoritmo clásico de fracción continua (Lentz), y con
# la simetría I_x(a,b) = 1 − I_{1−x}(b,a) converge en pocas iteraciones.

def _betacf(a, b, x, itmax=200, eps=3e-7):
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c, d = 1.0, 1.0 - qab * x / qap
    if abs(d) < 1e-30:
        d = 1e-30
    d = 1.0 / d
    h = d
    for m in range(1, itmax + 1):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        c = 1.0 + aa / c
        if abs(d) < 1e-30:
            d = 1e-30
        if abs(c) < 1e-30:
            c = 1e-30
        d = 1.0 / d
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        c = 1.0 + aa / c
        if abs(d) < 1e-30:
            d = 1e-30
        if abs(c) < 1e-30:
            c = 1e-30
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < eps:
            break
    return h


def _betai(a, b, x):
    """Beta incompleta regularizada I_x(a,b)."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    lbeta = (math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b)
             + a * math.log(x) + b * math.log(1.0 - x))
    front = math.exp(lbeta)
    if x < (a + 1.0) / (a + b + 2.0):
        return front * _betacf(a, b, x) / a
    return 1.0 - front * _betacf(b, a, 1.0 - x) / b


def one_way_anova(groups):
    """ANOVA de una vía sobre {etiqueta: [valores]}. Retorna dict con F, gl y p.

    Compara la varianza ENTRE escenarios contra la varianza DENTRO de cada uno:
    si los escenarios no influyeran, ambas estimarían lo mismo y F≈1.
    """
    gs = [v for v in groups.values() if len(v) >= 2]
    if len(gs) < 2:
        return None
    n = sum(len(v) for v in gs)
    k = len(gs)
    grand = sum(sum(v) for v in gs) / n
    ss_between = sum(len(v) * (sum(v) / len(v) - grand) ** 2 for v in gs)
    ss_within = sum((x - sum(v) / len(v)) ** 2 for v in gs for x in v)
    df1, df2 = k - 1, n - k
    if df2 <= 0 or ss_within <= 0:
        return None
    F = (ss_between / df1) / (ss_within / df2)
    p = _betai(df2 / 2.0, df1 / 2.0, df2 / (df2 + df1 * F))
    return {'F': F, 'df1': df1, 'df2': df2, 'p': p, 'n': n, 'k': k}


def _gammaln(x):
    """log Γ(x) — Lanczos, suficiente para los p-valores de χ²."""
    g = [676.5203681218851, -1259.1392167224028, 771.32342877765313,
         -176.61502916214059, 12.507343278686905, -0.13857109526572012,
         9.9843695780195716e-6, 1.5056327351493116e-7]
    if x < 0.5:
        return math.log(math.pi / math.sin(math.pi * x)) - _gammaln(1 - x)
    x -= 1
    a = 0.99999999999980993
    t = x + 7.5
    for i, c in enumerate(g):
        a += c / (x + i + 1)
    return 0.5 * math.log(2 * math.pi) + (x + 0.5) * math.log(t) - t + math.log(a)


def _chi2_sf(x, k):
    """P(χ²_k > x) — serie para x < k+1, fracción continua para el resto."""
    if x <= 0:
        return 1.0
    a, xx = k / 2.0, x / 2.0
    if xx < a + 1:
        term = 1.0 / a
        s, n = term, 0
        while abs(term) > abs(s) * 1e-12 and n < 500:
            n += 1
            term *= xx / (a + n)
            s += term
        return 1.0 - s * math.exp(-xx + a * math.log(xx) - _gammaln(a))
    b, c = xx + 1 - a, 1e30
    d = 1.0 / b
    h = d
    for i in range(1, 500):
        an = -i * (i - a)
        b += 2
        d = an * d + b
        if abs(d) < 1e-30:
            d = 1e-30
        c = b + an / c
        if abs(c) < 1e-30:
            c = 1e-30
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1) < 1e-12:
            break
    return math.exp(-xx + a * math.log(xx) - _gammaln(a)) * h


def levene(groups):
    """Prueba de Levene (centrada en la mediana) de homogeneidad de varianzas.

    ANOVA supone que todos los grupos tienen la misma varianza. En estos
    experimentos NO se cumple —un escenario puede ir de 21 a 179s y otro de 25 a
    36— así que antes de reportar un ANOVA hay que mirar esto. Se usa la variante
    de Brown-Forsythe (desviaciones respecto de la MEDIANA), que es la robusta
    cuando los datos no son normales.
    """
    g = [list(v) for v in groups.values() if len(v) > 1]
    k = len(g)
    if k < 2:
        return None
    z = [[abs(x - _median(v)) for x in v] for v in g]
    n = sum(len(v) for v in z)
    zbar = [sum(v) / len(v) for v in z]
    ztot = sum(sum(v) for v in z) / n
    num = sum(len(v) * (m - ztot) ** 2 for v, m in zip(z, zbar)) / (k - 1)
    den = sum((x - m) ** 2 for v, m in zip(z, zbar) for x in v) / (n - k)
    if den == 0:
        return None
    W = num / den
    return {'W': W, 'df1': k - 1, 'df2': n - k,
            'p': _betai((n - k) / 2.0, (k - 1) / 2.0,
                        (n - k) / ((n - k) + (k - 1) * W))}


def kruskal_wallis(groups):
    """Kruskal-Wallis: la alternativa no paramétrica al ANOVA de una vía.

    Trabaja sobre RANGOS, así que no supone normalidad ni varianzas iguales y
    aguanta los valores extremos que deja una corrida fallida. Es la prueba
    adecuada para este diseño: pocas repeticiones, varianzas dispares y
    distribuciones asimétricas.
    """
    g = [list(v) for v in groups.values() if v]
    k = len(g)
    if k < 2:
        return None
    todos = sorted((x, i) for i, v in enumerate(g) for x in v)
    n = len(todos)
    rangos = [0.0] * n
    i = 0
    while i < n:                      # promedio de rangos en los empates
        j = i
        while j + 1 < n and todos[j + 1][0] == todos[i][0]:
            j += 1
        r = (i + j) / 2.0 + 1
        for t in range(i, j + 1):
            rangos[t] = r
        i = j + 1
    suma = [0.0] * k
    for (x, gi), r in zip(todos, rangos):
        suma[gi] += r
    H = 12.0 / (n * (n + 1)) * sum(s * s / len(v) for s, v in zip(suma, g)) \
        - 3 * (n + 1)
    # corrección por empates
    emp = {}
    for x, _ in todos:
        emp[x] = emp.get(x, 0) + 1
    ties = sum(t ** 3 - t for t in emp.values() if t > 1)
    if ties and n > 1:
        H /= 1 - ties / (n ** 3 - n)
    return {'H': H, 'df': k - 1, 'p': _chi2_sf(H, k - 1), 'n': n, 'k': k}


def print_group_test(groups):
    """Compara los grupos y reporta la prueba que corresponda.

    Primero Levene: si las varianzas NO son homogéneas, el ANOVA de una vía no
    aplica y el estadístico que vale es Kruskal-Wallis, que trabaja sobre rangos
    y no supone ni normalidad ni varianzas iguales.

    Esto no es una preferencia de estilo. En este experimento las varianzas
    difieren de verdad —un escenario va de 21 a 179s y otro de 25 a 36— y por eso
    el paper reporta Kruskal-Wallis. Hasta el 2026-08-01 esta salida imprimía
    solo el ANOVA, o sea justo el estadístico que el artículo descarta y por la
    razón que lo descarta: quien reprodujera el comando obtenía un número que
    contradecía el texto.

    Cuando Levene no rechaza se imprimen los dos, y ojo que NO siempre coinciden:
    sobre el ratio de ruta del laboratorio las varianzas salen homogéneas
    (p=0.20) y aun así el ANOVA no ve diferencia entre escenarios (p=0.15)
    mientras Kruskal-Wallis sí (p=0.016). El ratio es una razón acotada por abajo
    en 1 y con cola larga a la derecha, y la prueba de rangos aguanta esa asimetría
    mucho mejor que la paramétrica. Ante desacuerdo manda Kruskal-Wallis, que es
    la que supone menos.
    """
    lev = levene(groups)
    kw = kruskal_wallis(groups)
    if kw is None:
        print('  insuficientes datos')
        return

    def estrellas(p):
        return ('***' if p < 0.001 else '**' if p < 0.01 else
                '*' if p < 0.05 else 'n.s.')

    heterogenea = lev is not None and lev['p'] < 0.05
    if lev is not None:
        print(f'  Levene W({lev["df1"]},{lev["df2"]})={lev["W"]:.2f} '
              f'p={lev["p"]:.4g} → varianzas '
              + ('DISPARES, el ANOVA no aplica' if heterogenea else 'homogéneas'))

    print(f'  Kruskal-Wallis H({kw["df"]})={kw["H"]:.2f}  p={kw["p"]:.4g}  '
          f'{estrellas(kw["p"])}   (n={kw["n"]}, k={kw["k"]})')

    if not heterogenea:
        an = one_way_anova(groups)
        if an is not None:
            print(f'  ANOVA F({an["df1"]},{an["df2"]})={an["F"]:.2f}  '
                  f'p={an["p"]:.4g}  {estrellas(an["p"])}   (control)')


def load_manifest(path):
    """Manifiesto de campaña: CSV con columnas
    session,scenario[,arranque][,scenario_json].

    Es explícito a propósito. Inferir el escenario del nombre del log haría que
    un renombre silencioso reasignara corridas a otra condición experimental.

    `arranque` es el segundo factor del diseño del lab (ASop/APar/ALin): la
    aleatorización de poses iniciales que pide el protocolo. Se arrastra para
    poder verificar que NO explica la varianza — si la explicara, las tres
    corridas de un escenario no serían repeticiones intercambiables y el ANOVA
    por escenario estaría mal planteado.
    """
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            if not r.get('session') or not r.get('scenario'):
                continue
            rows.append({'session': r['session'].strip(),
                         'scenario': r['scenario'].strip(),
                         'arranque': (r.get('arranque') or '').strip(),
                         'target': (r.get('target') or '').strip(),
                         'scenario_json': (r.get('scenario_json') or '').strip()})
    return rows


def run_campaign(a):
    """Consolida las corridas de la campaña y produce las figuras del paper.

    Una fila por corrida (tiempo de congregación, compactación) y una por robot
    por corrida (ratio de ruta), más el ANOVA de una vía sobre el escenario.
    """
    manifest = load_manifest(a.campaign)
    if not manifest:
        print(f'✗ Manifiesto vacío o ilegible: {a.campaign}')
        return
    available = {tag: (pos, con) for tag, pos, con
                 in find_sessions(os.path.dirname(os.path.abspath(a.campaign)))}

    runs, robots, missing = [], [], []
    scen_meta, series_by_scen = {}, {}
    for entry in manifest:
        match = [t for t in available if entry['session'] in t]
        if not match:
            missing.append(entry['session'])
            continue
        tag = match[-1]
        pos, con = available[tag]
        # El punto de destino sale del manifiesto (columna `target`) y --target
        # lo pisa para toda la campaña. Sin él el criterio va contra el
        # centroide, que en un experimento con destino fijo premia al enjambre
        # que se atascó antes de la barrera.
        tgt = None
        raw = a.target or entry['target']
        if raw:
            try:
                tx, ty = (float(v) for v in raw.split(','))
                tgt = (tx, ty)
            except ValueError:
                pass
        m = congregation_metrics(tag, pos, con, radius=a.radius, hold_s=a.hold,
                                 frac=a.frac, target=tgt)
        if m is None:
            missing.append(entry['session'])
            continue
        sc = entry['scenario']

        # métricas del escenario, si el sidecar de gen_world está disponible
        if entry['scenario_json'] and sc not in scen_meta:
            try:
                with open(entry['scenario_json']) as f:
                    scen_meta[sc] = json.load(f)
            except (OSError, ValueError):
                pass
        meta = scen_meta.get(sc, {})

        ph = m.get('phases') or {}
        runs.append({
            'session': tag, 'scenario': sc, 'arranque': entry['arranque'],
            'n_robots': m['n_robots'],
            't_conv_frac_s': m['t_conv_frac_s'],
            't_conv_all_s': m['t_conv_s'],
            'converged_frac': int(bool(m['converged_frac'])),
            'arrived': m['arrived'],
            'centroid_to_target_mm': m['centroid_to_target_mm'],
            'final_compaction_d': m['final_compaction_d'],
            'final_rms_d': m['final_rms_d'],
            # Fases: sin esto la única duración disponible es el largo del log,
            # que mezcla la caminata aleatoria y el tiempo del operador.
            'rw_s': ph.get('rw_s'),
            'dead_s': ph.get('dead_s'),
            'meet_s': ph.get('meet_s'),
            'log_s': ph.get('log_s'),
            'occupancy_pct': meta.get('occupancy_pct'),
            'obstacle_area_d': (meta.get('obstacle_area_d') or [None])[0],
            # El pasaje que importa es el EFECTIVO (cuello de botella de la ruta
            # más holgada). `passage_d` es el mínimo sobre TODOS los pares y
            # paredes, y encuentra rincones apretados fuera de camino: en la
            # campaña del 31-07 daba 0.57d para los escenarios de 2 diámetros y
            # 0.29d para los de 4, o sea al revés de como están diseñados.
            'passage_d': (round(meta['effective_passage_mm'] / ATTA_DIAMETER_MM, 2)
                          if meta.get('effective_passage_mm') else None),
            'passage_min_d': meta.get('passage_d'),
        })
        rp = m.get('robot_phases', {})
        for rid, p in m['per_robot'].items():
            if p['route_ratio'] is None:
                continue      # líder / robot que no se desplazó
            f = rp.get(rid, {})
            robots.append({'session': tag, 'scenario': sc,
                           'arranque': entry['arranque'], 'robot': rid,
                           'path_mm': p['path_mm'], 'straight_mm': p['straight_mm'],
                           'route_ratio': p['route_ratio'],
                           'evasions': p['evasions'],
                           'rw_active_s': f.get('rw_active_s'),
                           'meet_active_s': f.get('meet_active_s'),
                           't_stop_s': f.get('t_stop_s')})
        series_by_scen.setdefault(sc, []).append(m['series'])

    if missing:
        print(f'⚠ {len(missing)} corrida(s) del manifiesto sin log utilizable: '
              f'{", ".join(missing[:5])}{" …" if len(missing) > 5 else ""}')
    if not runs:
        print('✗ Ninguna corrida utilizable.')
        return

    order = sorted({r['scenario'] for r in runs})
    print(f'\n=== CAMPAÑA: {len(runs)} corridas, {len(order)} escenarios ===')
    print(f'{"escenario":<16}{"n":>3}{"RW(s)":>8}{"muerto":>8}{"MEET(s)":>9}'
          f'{"t_90%(s)":>10}{"compact(d)":>12}{"ruta":>8}{"ocup%":>7}'
          f'{"pasaje(d)":>10}')
    for sc in order:
        rr = [r for r in runs if r['scenario'] == sc]
        ts = [r['t_conv_frac_s'] for r in rr if r['t_conv_frac_s'] is not None]
        cs = [r['final_compaction_d'] for r in rr]
        rt = [x['route_ratio'] for x in robots if x['scenario'] == sc]
        meta = scen_meta.get(sc, {})

        def med(key):
            v = [r[key] for r in rr if r.get(key) is not None]
            return _median(v) if v else float('nan')

        print(f'{sc:<16}{len(rr):>3}'
              f'{med("rw_s"):>8.1f}{med("dead_s"):>8.1f}{med("meet_s"):>9.1f}'
              f'{(_median(ts) if ts else float("nan")):>10.1f}'
              f'{_median(cs):>12.2f}{(_median(rt) if rt else float("nan")):>8.2f}'
              f'{(meta.get("occupancy_pct") or float("nan")):>7.2f}'
              f'{(_median([r["passage_d"] for r in rr if r.get("passage_d")]) if any(r.get("passage_d") for r in rr) else float("nan")):>10.2f}')
        if len(ts) < len(rr):
            print(f'{"":16}   ({len(rr) - len(ts)} sin converger — excluidas '
                  f'de la mediana de tiempo)')

    for label, groups in (
            ('tiempo de congregación (90%)',
             {sc: [r['t_conv_frac_s'] for r in runs
                   if r['scenario'] == sc and r['t_conv_frac_s'] is not None]
              for sc in order}),
            ('ratio de ruta',
             {sc: [x['route_ratio'] for x in robots if x['scenario'] == sc]
              for sc in order})):
        print(f'\n{label}')
        print_group_test(groups)

    # Control del segundo factor. Si el arranque saliera significativo, las
    # corridas de un mismo escenario no serían repeticiones intercambiables.
    arr = sorted({r['arranque'] for r in runs if r['arranque']})
    if len(arr) > 1:
        print('\ncontrol por arranque (tiempo 90%) — debería salir n.s.')
        print_group_test({x: [r['t_conv_frac_s'] for r in runs
                              if r['arranque'] == x
                              and r['t_conv_frac_s'] is not None]
                          for x in arr})

    stem = a.campaign_out or 'campana'
    with open(f'{stem}_runs.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(runs[0].keys()))
        w.writeheader()
        w.writerows(runs)
    with open(f'{stem}_robots.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(robots[0].keys()))
        w.writeheader()
        w.writerows(robots)
    print(f'\nDatos: {stem}_runs.csv · {stem}_robots.csv')

    if a.plot:
        try:
            plot_campaign(runs, robots, series_by_scen, order, stem)
            print(f'Figuras: {stem}_boxplots.png · {stem}_compactacion.png · '
                  f'{stem}_heatmaps.png')
        except ImportError:
            print('  (matplotlib no disponible — omito las figuras)')


def plot_campaign(runs, robots, series_by_scen, order, stem):
    """Las cuatro figuras del paper: boxplots de tiempo y de ruta, dispersión de
    compactación en t, y heatmaps de ocupación por escenario."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    # 1) boxplots de tiempo y de ruta
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    tdata = [[r['t_conv_frac_s'] for r in runs
              if r['scenario'] == sc and r['t_conv_frac_s'] is not None]
             for sc in order]
    rdata = [[x['route_ratio'] for x in robots if x['scenario'] == sc]
             for sc in order]
    for axi, data, title, ylab in (
            (ax[0], tdata, 'Tiempo de congregación (90% en el radio)', 's'),
            (ax[1], rdata, 'Ratio de ruta por robot', 'recorrido ÷ recta')):
        # set_xticklabels en vez del kwarg labels/tick_labels: el nombre cambió
        # entre versiones de matplotlib (3.11 ya no acepta 'labels')
        axi.boxplot([d if d else [float('nan')] for d in data])
        axi.set_xticks(range(1, len(order) + 1))
        axi.set_xticklabels(order)
        axi.set_title(title)
        axi.set_ylabel(ylab)
        axi.grid(alpha=0.3)
        axi.tick_params(axis='x', rotation=20)
    ax[1].axhline(1.0, color='g', ls='--', lw=1, label='trayecto directo')
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(f'{stem}_boxplots.png', dpi=130)
    plt.close(fig)

    # 2) compactación en t: una curva por corrida, coloreada por escenario
    fig, axc = plt.subplots(figsize=(9, 5))
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(order), 2)))
    for i, sc in enumerate(order):
        for j, series in enumerate(series_by_scen.get(sc, [])):
            if not series:
                continue
            t0 = series[0]['t']
            axc.plot([s['t'] - t0 for s in series],
                     [s['compaction_d'] for s in series],
                     color=colors[i], alpha=0.55, lw=1.2,
                     label=sc if j == 0 else None)
    axc.set_xlabel('t desde el inicio del episodio (s)')
    axc.set_ylabel('compactación  √Σd²  (diámetros de Atta)')
    axc.set_title('Compactación del enjambre en el tiempo')
    axc.grid(alpha=0.3)
    axc.legend()
    fig.tight_layout()
    fig.savefig(f'{stem}_compactacion.png', dpi=130)
    plt.close(fig)

    # 3) heatmaps de ocupación por escenario (todas las poses de todas las corridas)
    n = len(order)
    fig, axh = plt.subplots(1, n, figsize=(4.2 * n, 3.6), squeeze=False)
    for i, sc in enumerate(order):
        xs, ys = [], []
        for series in series_by_scen.get(sc, []):
            for s in series:
                for (px, py) in s['pos'].values():
                    xs.append(px)
                    ys.append(py)
        axi = axh[0][i]
        if xs:
            axi.hexbin(xs, ys, gridsize=28, cmap='inferno', mincnt=1)
            axi.invert_yaxis()      # marco de cámara: y hacia abajo
        axi.set_title(sc, fontsize=10)
        axi.set_aspect('equal')
    fig.suptitle('Ocupación espacial por escenario')
    fig.tight_layout()
    fig.savefig(f'{stem}_heatmaps.png', dpi=130)
    plt.close(fig)


def plot_congregation(m, out_path, layout=None, obstacles=None):
    """Figura de 3 paneles: trayectorias, spread vs t (con R y T_conv), y
    dist. mínima inter-robot vs t. Requiere matplotlib."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    series = m['series']
    ts = [s['t'] for s in series]
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))

    # (1) trayectorias
    rids = sorted(m['per_robot'], key=lambda r: int(r))
    for rid in rids:
        xs = [s['pos'][rid][0] for s in series if rid in s['pos']]
        ys = [s['pos'][rid][1] for s in series if rid in s['pos']]
        ax[0].plot(xs, ys, '-', lw=1.2, label=f'Atta_{rid}')
        if xs:
            ax[0].plot(xs[0], ys[0], 'o', ms=6)
            ax[0].plot(xs[-1], ys[-1], 's', ms=7)
    for ox, oy, *rest in (obstacles or []):
        w = rest[0] if rest else 150
        ax[0].add_patch(plt.Rectangle((ox - w / 2, oy - w / 2), w, w,
                                      color='0.5', alpha=0.6))
    cx, cy = m['final_centroid']
    ax[0].plot(cx, cy, 'k*', ms=12, label='centroide')
    ax[0].set_title(f"Trayectorias{'  ['+layout+']' if layout else ''}")
    ax[0].set_xlabel('x (mm)'); ax[0].set_ylabel('y (mm)')
    ax[0].set_aspect('equal', 'box'); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)

    # (2) spread (radio del grupo) vs t
    ax[1].plot(ts, [s['spread'] for s in series], '-', color='C3')
    ax[1].axhline(m['radius_mm'], ls='--', color='0.4', label=f"R={m['radius_mm']:.0f}mm")
    if m['converged']:
        ax[1].axvline(m['t_conv_abs_s'], ls=':', color='C2',
                      label=f"T_conv={m['t_conv_s']:.1f}s")
    ax[1].set_title('Radio del grupo (max dist. al centroide)')
    ax[1].set_xlabel('t (s)'); ax[1].set_ylabel('spread (mm)')
    ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)

    # (3) distancia mínima inter-robot vs t
    ax[2].plot(ts, [s['min_pair'] for s in series], '-', color='C0')
    ax[2].set_title('Distancia mínima inter-robot')
    ax[2].set_xlabel('t (s)'); ax[2].set_ylabel('min pairwise (mm)')
    ax[2].grid(alpha=0.3)

    fig.suptitle(f"Congregación — {m['session']}", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path


def analyze_session(tag, pos_path, con_path, verbose=True):
    """Métricas de una sesión. Retorna lista de runs (dicts)."""
    if con_path is None:
        if verbose:
            print(f'{tag}: sin ConsoleLog pareado — no se pueden segmentar runs')
        return []
    events = load_console(con_path)
    tracks = load_positions(pos_path)
    all_runs = []
    for rid in sorted(tracks):
        for run in segment_runs(events, rid):
            m = run_metrics(run, tracks[rid])
            if m is None:
                continue
            m['robot'] = rid
            m['session'] = tag
            all_runs.append(m)

    if verbose and all_runs:
        print(f'\n=== {tag} — {len(all_runs)} run(s) ===')
        hdr = (f'{"robot":>5} {"tipo":<13} {"llegó":>5} {"dur(s)":>7} '
               f'{"pasos":>5} {"camino":>7} {"recta":>6} {"efic":>5} '
               f'{"err_fin":>7} {"evas":>4} {"obst":>4}')
        print(hdr)
        for m in all_runs:
            err = f"{m['final_error_mm']:.0f}" if m['final_error_mm'] is not None else '—'
            print(f"{m['robot']:>5} {m['kind']:<13} "
                  f"{'sí' if m['arrived'] else 'NO':>5} {m['duration_s']:>7} "
                  f"{m['steps']:>5} {m['path_mm']:>7.0f} {m['straight_mm']:>6.0f} "
                  f"{m['efficiency']:>5} {err:>7} {m['evasions']:>4} "
                  f"{m['obstacle_events']:>4}")
    elif verbose:
        print(f'{tag}: sin runs de navegación detectados')
    return all_runs


def summarize(runs):
    """Resumen agregado (mediana de las métricas de los runs que llegaron)."""
    ok = [r for r in runs if r['arrived']]
    if not runs:
        return
    print(f'\n=== RESUMEN: {len(runs)} runs, {len(ok)} llegadas '
          f'({100 * len(ok) / len(runs):.0f}%) ===')
    if not ok:
        return

    print(f"convergencia mediana : {_median([r['duration_s'] for r in ok]):.1f} s")
    print(f"pasos medianos       : {_median([r['steps'] for r in ok]):.0f}")
    print(f"eficiencia mediana   : {_median([r['efficiency'] for r in ok]):.2f}")
    errs = [r['final_error_mm'] for r in ok if r['final_error_mm'] is not None]
    if errs:
        print(f"error final mediano  : {_median(errs):.0f} mm (n={len(errs)})")
    print(f"evasiones totales    : {sum(r['evasions'] for r in runs)}")


def run_congregation(sessions, a):
    """Modo grupal: métricas de congregación por sesión (tiempo de convergencia,
    centroide, dist. inter-robot, y recorrido por robot)."""
    rows = []       # una fila por robot por sesión (para el CSV del paper)
    for tag, pos, con in sessions:
        m = congregation_metrics(tag, pos, con, radius=a.radius, hold_s=a.hold,
                                 frac=a.frac)
        if m is None:
            if not a.all:
                print(f'{tag}: sin ≥2 robots con trayectoria — no aplica congregación')
            continue
        if not a.all:
            print_congregation(m, layout=a.layout)
        if a.plot:
            try:
                out = os.path.join(BASE_DIR, f'congreg_{tag}.png')
                plot_congregation(m, out, layout=a.layout)
                print(f'  figura → {out}')
            except ImportError:
                print('  (matplotlib no disponible — omito la figura)')
        for rid, p in m['per_robot'].items():
            rows.append({
                'session': tag, 'layout': a.layout or '',
                'robot': rid, 'is_leader': int(rid == m['leader']),
                'n_robots': m['n_robots'], 'radius_mm': m['radius_mm'],
                'converged': int(m['converged']), 't_conv_s': m['t_conv_s'],
                'robot_path_mm': p['path_mm'], 'robot_efficiency': p['efficiency'],
                'robot_final_x': p['final'][0], 'robot_final_y': p['final'][1],
                'robot_evasions': p['evasions'],
                'final_spread_mm': m['final_spread_mm'],
                'final_min_pair_mm': m['final_min_pair_mm'],
                'min_pair_ever_mm': m['min_pair_ever_mm'],
            })
        if a.all:
            conv = f"{m['t_conv_s']:.1f}s" if m['converged'] else 'NO'
            print(f"{tag:<34} {m['n_robots']} rob  conv={conv:<7} "
                  f"spread_fin={m['final_spread_mm']:>5}mm  "
                  f"min_pair={m['min_pair_ever_mm']:>5}mm")

    if a.csv and rows:
        cols = ['session', 'layout', 'robot', 'is_leader', 'n_robots',
                'radius_mm', 'converged', 't_conv_s', 'robot_path_mm',
                'robot_efficiency', 'robot_final_x', 'robot_final_y',
                'robot_evasions', 'final_spread_mm', 'final_min_pair_mm',
                'min_pair_ever_mm']
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
            w.writeheader()
            w.writerows(rows)
        print(f'\nMétricas de congregación exportadas a {a.csv}')


# ── Calibración / prueba de IR (Fase 1) ──────────────────────────────────────
# El log de CHECK_OBSTACLE ya trae la pose del robot al disparar:
#   CHECK_OBSTACLE|<bitmap>|<x>|<y>|<angle>
# → cada disparo dice QUÉ canales se activaron y DÓNDE estaba el robot, así que
# la distancia de detección y los fantasmas salen del propio log (sin cámara).
def ir_channels(bitmap):
    """bitmap (4·IZQ+2·CEN+1·DER) → lista de canales activos."""
    return [name for bit, name in IR_BITS if bitmap & bit]


def load_check_obstacle(path):
    """[(t, robot_id, bitmap, x, y, ang)] del ConsoleLog. Tolera el formato
    viejo sin pose (x=y=ang=None)."""
    out = []
    for t, rid, msg in load_console(path):
        if not msg.startswith('CHECK_OBSTACLE'):
            continue
        parts = msg.split('|')
        try:
            bm = int(float(parts[1]))
        except (IndexError, ValueError):
            continue
        x = y = ang = None
        if len(parts) >= 5:
            try:
                x, y, ang = float(parts[2]), float(parts[3]), float(parts[4])
            except ValueError:
                x = y = ang = None
        out.append((t, rid, bm, x, y, ang))
    return out


def print_ir(tag, events, obstacles):
    print(f'\n=== CALIBRACIÓN IR  {tag} ===')
    if obstacles:
        print('obstáculo(s) (mm): ' +
              '  '.join(f'({x:.0f},{y:.0f})' for x, y in obstacles))
        print(f'distancia = centro del robot → obstáculo más cercano; '
              f'fantasma = disparo a >{IR_GHOST_DIST:.0f}mm de todo obstáculo')
    else:
        print('modo LIBRE (sin --obstacle): se asume arena vacía ⇒ TODO disparo '
              'IR es un fantasma')
    robots = sorted({e[1] for e in events},
                    key=lambda r: int(r) if r.isdigit() else 1e9)
    print(f'\n{"robot":<8}{"canal":<6}{"disparos":>9}{"cen→obst med/min":>18}'
          f'{"fantasmas":>11}  recomendación')
    for rid in robots:
        rev = [e for e in events if e[1] == rid]
        for ch in ('IZQ', 'CEN', 'DER'):
            fires = [e for e in rev if ch in ir_channels(e[2])]
            if not fires:
                continue
            dists, ghosts, nopose = [], 0, 0
            for (_t, _r, _bm, x, y, _ang) in fires:
                if x is None:
                    nopose += 1
                    continue
                if obstacles:
                    d = min(math.dist((x, y), o) for o in obstacles)
                    (dists.append(d) if d <= IR_GHOST_DIST else None)
                    ghosts += d > IR_GHOST_DIST
                else:
                    ghosts += 1
            det = f'{_median(dists):.0f}/{min(dists):.0f}' if dists else '—'
            rec = ''
            if ghosts >= 5 and ghosts >= 0.6 * (len(fires) - nopose or 1):
                rec = f'SENSOR_MASK|{IR_MASK[ch]}|1  (fantasma {ghosts}/{len(fires)})'
            elif not dists and obstacles and ch in ('IZQ', 'DER'):
                rec = '¿sensor muerto? nunca cerca del obstáculo'
            print(f'{"Atta_" + rid:<8}{ch:<6}{len(fires):>9}{det:>18}'
                  f'{ghosts:>11}  {rec}')
    note = ('CEN = proximidad APDS9960 (no IR). Un fantasma persistente en IZQ/DER '
            'se enmascara con la recomendación; re-enviar tras CALIBRATE.')
    print(f'\n  nota: {note}')


def load_ekf_pairs(path, max_age_ms=600.0):
    """
    {robot_id: [(t, cam_x, cam_y, cam_ang, ekf_x, ekf_y, ekf_ang)]}

    Solo filas donde el PositionLog trae las columnas ekf_* (logs viejos no las
    tienen) y donde la muestra del EKF es fresca: llega a 2Hz mientras el log se
    escribe por frame, así que una muestra vieja compararía el EKF de hace un
    segundo contra la cámara de ahora e inflaría el error sin que sea deriva.
    """
    pairs = {}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            if not row.get('ekf_x'):
                continue
            try:
                age = float(row['ekf_age_ms'])
                if age > max_age_ms:
                    continue
                pairs.setdefault(row['idrobot'], []).append(
                    (float(row['time']),
                     float(row['x']), float(row['y']), float(row['angle']),
                     float(row['ekf_x']), float(row['ekf_y']),
                     float(row['ekf_angle'])))
            except (ValueError, KeyError, TypeError):
                continue
    return pairs


def print_ekf(tag, pairs):
    """
    Error del EKF contra el ArUco: cuánto se le puede confiar la navegación.

    El EKF corre como observador pasivo (EKF_NAV apagado), así que esto mide sin
    arriesgar nada. Lo que importa para decidir es el p95, no el promedio: la
    navegación falla en el peor caso, no en el típico.
    """
    print(f'\n=== EKF vs ArUco — {tag} ===')
    print('El EKF es pasivo (EKF_NAV apagado): esto mide su deriva sin que controle.')
    hdr = (f'{"robot":>6} {"n":>5} {"med mm":>8} {"p95 mm":>8} {"máx mm":>8} '
           f'{"p95 áng":>8}')
    print(hdr)
    print('-' * len(hdr))

    todos = []
    for rid in sorted(pairs, key=lambda r: (len(r), r)):
        rows = pairs[rid]
        errs, aerrs = [], []
        for _t, cx, cy, ca, ex, ey, ea in rows:
            errs.append(math.hypot(ex - cx, ey - cy))
            d = abs((ea - ca + 180.0) % 360.0 - 180.0)
            aerrs.append(d)
        if not errs:
            continue
        errs.sort(); aerrs.sort()
        p95 = errs[min(len(errs) - 1, int(len(errs) * 0.95))]
        a95 = aerrs[min(len(aerrs) - 1, int(len(aerrs) * 0.95))]
        todos += errs
        print(f'{rid:>6} {len(errs):>5} {_median(errs):>8.0f} {p95:>8.0f} '
              f'{errs[-1]:>8.0f} {a95:>7.1f}°')

    if not todos:
        print('\nSin filas comparables. ¿La corrida es anterior al logueo del EKF, '
              'o los robots nunca llegaron a inicializarlo (necesita ArUco al menos '
              'una vez)?')
        return

    todos.sort()
    p95 = todos[min(len(todos) - 1, int(len(todos) * 0.95))]
    print(f'\nGlobal: mediana {_median(todos):.0f}mm · p95 {p95:.0f}mm · '
          f'máx {todos[-1]:.0f}mm  (n={len(todos)})')
    # El umbral de decisión es el criterio de llegada de la nav: si el EKF se
    # equivoca más que eso, navegar con él haría fallar la llegada.
    print('Criterio: el p95 tiene que quedar bajo arrivalThreshold (50mm) para '
          'confiarle la navegación;')
    print('          hasta ~150mm sirve solo para puentear oclusiones cortas.')
    if p95 <= 50:
        print(f'→ p95 {p95:.0f}mm: apto para EKF_NAV|1.')
    elif p95 <= 150:
        print(f'→ p95 {p95:.0f}mm: sirve de respaldo ante oclusión, no para navegar todo el tiempo.')
    else:
        print(f'→ p95 {p95:.0f}mm: NO confiarle la navegación todavía — revisar '
              'calibración de PPR/yaw_scale antes que el EKF.')


def run_ekf(sessions, a):
    visto = False
    for tag, pos, _con in sessions:
        pairs = load_ekf_pairs(pos)
        if not pairs:
            continue
        visto = True
        print_ekf(tag, pairs)
    if not visto:
        print('Ningún PositionLog trae columnas ekf_*. Se agregaron el 2026-07-29: '
              'las corridas anteriores no las tienen.')


def run_ir(sessions, a):
    obstacles = []
    for spec in (a.obstacle or []):
        xs = spec.split(',')
        try:
            obstacles.append((float(xs[0]), float(xs[1])))
        except (IndexError, ValueError):
            print(f'--obstacle inválido: "{spec}" (esperado x,y en mm)')
            return
    any_ev = False
    for tag, _pos, con in sessions:
        if con is None:
            continue
        events = load_check_obstacle(con)
        if not events:
            continue
        any_ev = True
        print_ir(tag, events, obstacles)
    if not any_ev:
        print('Sin eventos CHECK_OBSTACLE en la(s) sesión(es). ¿El robot navegó '
              'hacia el obstáculo con el debug de obstáculos activo?')


def visibility(pos_path):
    """{rid: {'vis_pct', 'gap_max_s', 'gaps'}} — cuánto tiempo se vio cada marker.

    El PositionLog solo escribe fila cuando el robot se movió ≥4mm/4°, así que un
    hueco NO prueba pérdida de marker. Se cuenta como pérdida solo si el robot
    reaparece en otro lado (>60mm): quieto no deja filas, pero tampoco se mueve.
    """
    tracks = real_tracks(load_positions(pos_path))
    if not tracks:
        return {}
    tmax = max(p[0] for t in tracks.values() for p in t)
    out = {}
    for rid, track in tracks.items():
        track = sorted(track)
        seen = {int(p[0]) for p in track}
        gaps = []
        for a, b in zip(track, track[1:]):
            dt = b[0] - a[0]
            if dt >= 1.0 and math.dist((a[1], a[2]), (b[1], b[2])) > 60.0:
                gaps.append(dt)
        out[rid] = {'vis_pct': 100.0 * len(seen) / (int(tmax) + 1),
                    'gap_max_s': max(gaps) if gaps else 0.0,
                    'gaps': len(gaps)}
    return out


def run_vis(sessions, a):
    """Chequeo rápido post-corrida: ¿esta corrida sirve o hay que repetirla?

    Pensado para correrlo en el lab entre experimento y experimento, cuando lo
    único que importa es la decisión repetir/seguir — no las métricas finales.
    """
    for tag, pos, con in sessions:
        vis = visibility(pos)
        if not vis:
            print(f'\n{tag}: sin datos de posición')
            continue
        ekf = load_ekf_pairs(pos) if con else {}
        peor = min(v['vis_pct'] for v in vis.values())
        veredicto = ('OK' if peor >= 85 else
                     'ACEPTABLE' if peor >= 75 else
                     'REPETIR (marker perdido demasiado tiempo)')
        print(f'\n=== {tag} ===   peor visibilidad {peor:.0f}%  →  {veredicto}')
        print(f'{"robot":>7}{"visible":>9}{"huecos":>8}{"peor hueco":>12}'
              f'{"err EKF":>10}')
        for rid in sorted(vis, key=lambda r: int(r)):
            v = vis[rid]
            errs = [math.dist((cx, cy), (ex, ey))
                    for _t, cx, cy, _ca, ex, ey, _ea in ekf.get(rid, [])]
            err = f'{_median(errs):.0f}mm' if len(errs) >= 20 else '—'
            marca = '  ⚠' if v['vis_pct'] < 85 else ''
            print(f'  Atta_{rid:<2}{v["vis_pct"]:>8.0f}%{v["gaps"]:>8}'
                  f'{v["gap_max_s"]:>11.1f}s{err:>10}{marca}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('--session', help='tag parcial, ej. 03-07_10-04 o SIM')
    ap.add_argument('--all', action='store_true', help='todas las sesiones')
    ap.add_argument('--csv', help='exportar a un CSV')
    ap.add_argument('--congregation', action='store_true',
                    help='métricas de congregación grupal (centroide, convergencia,'
                         ' dist. inter-robot, recorrido por robot)')
    ap.add_argument('--radius', type=float, default=450.0,
                    help='radio de convergencia R en mm (spread ≤ R = congregados)')
    ap.add_argument('--hold', type=float, default=3.0,
                    help='segundos que el spread debe mantenerse ≤ R')
    ap.add_argument('--layout', help='etiqueta de topología de obstáculos del run')
    ap.add_argument('--campaign', metavar='MANIFIESTO',
                    help='modo campaña: CSV con session,scenario[,scenario_json] '
                         '— consolida los runs, saca boxplots/heatmaps y el ANOVA')
    ap.add_argument('--campaign-out', metavar='PREFIJO',
                    help='prefijo de los archivos de salida de la campaña')
    ap.add_argument('--target', metavar='X,Y',
                    help='punto de destino en mm: mide la convergencia contra ÉL '
                         'y no contra el centroide. Obligatorio en experimentos '
                         'con destino fijo (topología): sin esto, un enjambre '
                         'atascado antes de la barrera cuenta como convergido')
    ap.add_argument('--frac', type=float, default=0.9,
                    help='fracción de robots dentro de R para dar la congregación '
                         'por lograda (default 0.9 = criterio del paper)')
    ap.add_argument('--plot', action='store_true',
                    help='guardar figura congreg_<sesión>.png (requiere matplotlib)')
    ap.add_argument('--ir', action='store_true',
                    help='calibración/prueba de IR: distancia de detección por canal '
                         '(IZQ/CEN/DER) y fantasmas, desde CHECK_OBSTACLE')
    ap.add_argument('--vis', action='store_true',
                    help='chequeo rápido post-corrida: visibilidad del marker por '
                         'robot y veredicto repetir/seguir')
    ap.add_argument('--ekf', action='store_true',
                    help='error del EKF del firmware contra el ArUco (pasivo, no '
                         'necesita EKF_NAV activo): mediana/p95/máx por robot')
    ap.add_argument('--obstacle', nargs='*', metavar='X,Y',
                    help='posición(es) del obstáculo colocado (mm cámara) para el '
                         'modo --ir; sin esto, arena vacía ⇒ todo disparo = fantasma')
    a = ap.parse_args()

    sessions = find_sessions()
    if not sessions:
        print('No hay PositionLogs')
        return

    # La campaña resuelve sus propias sesiones desde el manifiesto, no desde
    # --session/--all: cada corrida viene emparejada con su escenario.
    if a.campaign:
        run_campaign(a)
        return

    if a.session:
        sessions = [s for s in sessions if a.session in s[0]]
        if not sessions:
            print(f'Ninguna sesión matchea "{a.session}"')
            return
    elif not a.all:
        sessions = sessions[-1:]   # la más reciente

    if a.vis:
        run_vis(sessions, a)
        return

    if a.ekf:
        run_ekf(sessions, a)
        return

    if a.ir:
        run_ir(sessions, a)
        return

    if a.congregation:
        run_congregation(sessions, a)
        return

    runs = []
    for tag, pos, con in sessions:
        runs.extend(analyze_session(tag, pos, con, verbose=not a.all))

    if a.all:
        # En modo --all: una línea por sesión con runs + el agregado global
        for tag in dict.fromkeys(r['session'] for r in runs):
            rs = [r for r in runs if r['session'] == tag]
            ok = sum(1 for r in rs if r['arrived'])
            print(f'{tag:<32} {len(rs):>3} runs  {ok:>3} llegadas')
    summarize(runs)

    if a.csv and runs:
        cols = ['session', 'robot', 'kind', 'arrived', 't0', 't1', 'duration_s',
                'steps', 'path_mm', 'straight_mm', 'efficiency',
                'final_error_mm', 'evasions', 'obstacle_events', 'goal']
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
            w.writeheader()
            w.writerows(runs)
        print(f'\nRuns exportados a {a.csv}')


if __name__ == '__main__':
    main()
