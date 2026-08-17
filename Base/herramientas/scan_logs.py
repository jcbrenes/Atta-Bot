#!/usr/bin/env python3
"""Escáner de anomalías sobre TODOS los logs del lab (ConsoleLogs+PositionLogs).
Busca: misreads de markers, teletransportes, jitter por marker, fantasmas IR
por robot/sesión, abortos/loops, y patrones raros. Salida: resumen agregado
+ top de anomalías con archivo de origen."""
import csv
import glob
import math
import os
import re
from collections import defaultdict, Counter

# Los scripts viven en un subdirectorio, asi que la carpeta Base — donde estan
# los logs, los paquetes de datos y configSystem.json — es la de arriba.
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 0 = marker de origen; 1..11 los robots (MAX_NEIGHBORS del firmware + 1). Con
# el tope viejo de 4, la campana de simulacion de 10 robots reportaba seis
# 'markers fantasma' con 54 mil detecciones cada uno, que eran robots legitimos.
KNOWN_IDS = {str(i) for i in range(12)}


def etiqueta(ruta):
    """Como agrupar un log en la salida.

    Los de sesion se llaman Console_Log_SIM_31-07_16-20_Robots_10.csv y ahi la
    fecha es el campo util. Los del dataset del paper se renombraron al nombre
    del escenario (SinObs_ALin.csv), que tiene menos campos y ninguna fecha:
    tomar a ciegas el tercero reventaba con IndexError y el escaner no corria
    contra el dataset, que es justo donde interesa mirar.
    """
    partes = os.path.basename(ruta).rsplit('.', 1)[0].split('_')
    for p in partes:
        if re.fullmatch(r'\d{2}-\d{2}', p):
            return p
    return partes[0]

# ── ConsoleLogs ──────────────────────────────────────────────────────────────
con_events = defaultdict(Counter)      # fecha → contador de eventos
ghost = defaultdict(Counter)           # (robot, patrón IR) → count por fecha
oddities = []

for f in sorted(glob.glob(f'{BASE}/ConsoleLogs/*.csv')):
    fecha = etiqueta(f)
    try:
        rows = list(csv.reader(open(f, errors='replace')))
    except Exception as e:
        oddities.append(f'{f}: ilegible ({e})')
        continue
    for r in rows[1:]:
        if len(r) < 4:
            continue
        msg = r[3]
        rid = r[1]
        if msg.startswith('CHECK_OBSTACLE'):
            patt = msg.split('|')[1] if '|' in msg else '?'
            ghost[(rid, patt)][fecha] += 1
            con_events[fecha]['check_obstacle'] += 1
        elif 'Evasión' in msg or 'Evasion' in msg:
            con_events[fecha]['evasion'] += 1
        elif 'retroceso forzado' in msg.lower():
            con_events[fecha]['retroceso_forzado'] += 1
        elif 'LOOP DETECTADO' in msg:
            con_events[fecha]['loop_abort'] += 1
            oddities.append(f'{fecha} r{rid}: LOOP DETECTADO ({os.path.basename(f)})')
        elif 'Sin progreso' in msg:
            con_events[fecha]['sin_progreso'] += 1
        elif 'no visible' in msg.lower() or 'no disponible' in msg.lower():
            con_events[fecha]['no_visible'] += 1
        elif 'llegó' in msg:
            con_events[fecha]['llego'] += 1
        elif 'corrección #' in msg:
            con_events[fecha]['turn_corr'] += 1

# ── PositionLogs ─────────────────────────────────────────────────────────────
unknown_ids = Counter()                # id fantasma → count
unknown_where = defaultdict(set)
teleports = Counter()                  # fecha → count
teleport_worst = []
jitter = defaultdict(list)             # id → stds de ventanas quietas

for f in sorted(glob.glob(f'{BASE}/PositionLogs/*.csv')):
    fecha = etiqueta(f)
    per_id = defaultdict(list)
    try:
        with open(f, errors='replace') as fh:
            rd = csv.reader(fh)
            next(rd, None)
            for r in rd:
                if len(r) < 6:
                    continue
                try:
                    per_id[r[1]].append((float(r[0]), float(r[3]), float(r[4])))
                except ValueError:
                    continue
    except Exception as e:
        oddities.append(f'{f}: ilegible ({e})')
        continue

    for rid, pts in per_id.items():
        if rid not in KNOWN_IDS:
            unknown_ids[rid] += len(pts)
            unknown_where[rid].add(f'{fecha}({os.path.basename(f)[13:24]})')
            continue
        # teletransportes: salto enorme entre muestras consecutivas cercanas en t
        for i in range(1, len(pts)):
            dt = pts[i][0] - pts[i-1][0]
            d = math.hypot(pts[i][1]-pts[i-1][1], pts[i][2]-pts[i-1][2])
            if 0 < dt < 0.3 and d > 250:
                teleports[fecha] += 1
                if d > 500:
                    teleport_worst.append((d, fecha, rid))
        # jitter estacionario: ventanas de 10 muestras con recorrido corto
        W = 10
        for i in range(0, len(pts) - W, W):
            win = pts[i:i+W]
            xs = [p[1] for p in win]; ys = [p[2] for p in win]
            span = math.hypot(max(xs)-min(xs), max(ys)-min(ys))
            if span < 60:   # quieto-ish
                mx = sum(xs)/W; my = sum(ys)/W
                std = (sum((x-mx)**2 + (y-my)**2 for x, y in zip(xs, ys))/W) ** 0.5
                jitter[rid].append(std)

# ── Reporte ──────────────────────────────────────────────────────────────────
print('══════ MARKERS FANTASMA (ids no esperados) ══════')
for rid, n in unknown_ids.most_common(10):
    print(f'  id {rid}: {n} detecciones — sesiones: {sorted(unknown_where[rid])[:5]}')

print('\n══════ JITTER ESTACIONARIO POR MARKER (mm, mediana / p90) ══════')
for rid in sorted(jitter):
    v = sorted(jitter[rid])
    if len(v) >= 20:
        print(f'  marker {rid}: mediana={v[len(v)//2]:.1f}  p90={v[int(len(v)*0.9)]:.1f}  (n={len(v)} ventanas)')

print('\n══════ TELETRANSPORTES (saltos >250mm en <0.3s) por fecha ══════')
for fecha, n in sorted(teleports.items(), key=lambda kv: -kv[1])[:8]:
    print(f'  {fecha}: {n}')
worst = sorted(teleport_worst, reverse=True)[:5]
for d, fecha, rid in worst:
    print(f'  peor: {d:.0f}mm ({fecha}, robot {rid})')

print('\n══════ FANTASMAS IR: CHECK_OBSTACLE por robot y patrón (total histórico) ══════')
tot = Counter()
for (rid, patt), fechas in ghost.items():
    tot[(rid, patt)] = sum(fechas.values())
names = {'1': 'DER', '2': 'CEN', '4': 'IZQ', '3': 'C+D', '6': 'I+C', '7': 'TODOS', '5': 'I+D'}
for (rid, patt), n in tot.most_common(12):
    print(f'  robot {rid} sensor {names.get(patt, patt)}: {n}')

print('\n══════ EVENTOS DE CONTROL POR FECHA (últimas 8 sesiones-día) ══════')
fechas = sorted(con_events, key=lambda f: (f.split("-")[1], f.split("-")[0]))
for fecha in fechas[-8:]:
    c = con_events[fecha]
    print(f'  {fecha}: llegó={c["llego"]} evasiones={c["evasion"]} '
          f'retro_forzado={c["retroceso_forzado"]} no_visible={c["no_visible"]} '
          f'loops={c["loop_abort"]} sin_progreso={c["sin_progreso"]} '
          f'corr_turn={c["turn_corr"]}')

print('\n══════ RAREZAS SUELTAS ══════')
for o in oddities[:12]:
    print(f'  {o}')
print(f'  (total rarezas: {len(oddities)})')
