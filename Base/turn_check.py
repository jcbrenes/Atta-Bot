#!/usr/bin/env python3
"""
turn_check.py — Validación de giros y avances contra la cámara ArUco.

La cámara es el patrón de calibración. Sobre un PositionLog:
  - desenrolla el ángulo ArUco frame a frame (elimina saltos de 360°),
  - detecta MESETAS = robot QUIETO (baja varianza de ángulo Y de posición),
  - cada transición entre mesetas es un GIRO (Δángulo grande, Δpos chico)
    o un AVANCE (Δpos grande, Δángulo chico).

Detectar quietud por posición+ángulo (no solo ángulo) separa bien los avances:
durante un MOVE el ángulo queda quieto pero la posición no, así que un detector
por-ángulo solo fusionaría el avance con las mesetas vecinas.

Uso:
  python3 Base/turn_check.py <log.csv> <id> [--expected 360|180]
  python3 Base/turn_check.py                # log más reciente, robot 1, eventos

  --robot N         id del robot (default: 1)
  --expected DEG    resumen de giros cercanos a ese valor (error + escala)
  --std-thresh D    umbral varianza de ÁNGULO para "quieto" (default 5.0°)
  --pos-thresh D    umbral varianza de POSICIÓN para "quieto" mm (default 30)
  --win W           ventana del rolling std en muestras (default 7)
  --min-len N       muestras mínimas para que cuente como meseta (default 5)
"""
import sys
import csv
import glob
import os
import argparse
import numpy as np

LOGDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "PositionLogs")


def latest_log():
    files = glob.glob(os.path.join(LOGDIR, "*.csv"))
    if not files:
        sys.exit(f"No hay logs en {LOGDIR}")
    return max(files, key=os.path.getmtime)


def load(path, robot_id):
    t, ang, x, y = [], [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                if int(float(row["idrobot"])) != robot_id:
                    continue
                t.append(float(row["time"]))
                ang.append(float(row["angle"]))
                x.append(float(row["x"]))
                y.append(float(row["y"]))
            except (ValueError, KeyError):
                continue
    if not ang:
        sys.exit(f"No hay filas para robot id={robot_id} en {os.path.basename(path)}")
    return (np.array(t), np.degrees(np.unwrap(np.radians(np.array(ang)))),
            np.array(x), np.array(y))


def rolling_std(v, win):
    h = win // 2
    out = np.empty(len(v))
    for i in range(len(v)):
        out[i] = np.std(v[max(0, i - h):min(len(v), i + h + 1)])
    return out


def find_plateaus(ang, x, y, std_thresh, pos_thresh, win, min_len):
    """Regiones (i0, i1) donde el robot está QUIETO: ángulo y posición estables."""
    sa = rolling_std(ang, win)
    sx = rolling_std(x, win)
    sy = rolling_std(y, win)
    settled = (sa < std_thresh) & (sx < pos_thresh) & (sy < pos_thresh)
    plats, start = [], None
    for i, s in enumerate(settled):
        if s and start is None:
            start = i
        elif not s and start is not None:
            if i - start >= min_len:
                plats.append((start, i - 1))
            start = None
    if start is not None and len(ang) - start >= min_len:
        plats.append((start, len(ang) - 1))
    return plats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", default=None)
    ap.add_argument("robot", nargs="?", type=int, default=None)
    ap.add_argument("--robot", dest="robot_kw", type=int, default=None)
    ap.add_argument("--expected", type=float, default=None)
    ap.add_argument("--std-thresh", type=float, default=5.0)
    ap.add_argument("--pos-thresh", type=float, default=30.0)
    ap.add_argument("--win", type=int, default=7)
    ap.add_argument("--min-len", type=int, default=5)
    a = ap.parse_args()

    path = a.log or latest_log()
    robot = a.robot if a.robot is not None else (a.robot_kw if a.robot_kw is not None else 1)

    t, ang, x, y = load(path, robot)
    plats = find_plateaus(ang, x, y, a.std_thresh, a.pos_thresh, a.win, a.min_len)

    print(f"Log:   {os.path.basename(path)}")
    print(f"Robot: id={robot}   muestras={len(ang)}   mesetas={len(plats)}")
    if len(plats) < 2:
        print("⚠ Mesetas insuficientes. Probá --std-thresh/--pos-thresh mayores o --min-len menor.")
        return

    meds = []
    for (i0, i1) in plats:
        meds.append((float(np.median(ang[i0:i1 + 1])),
                     float(np.median(x[i0:i1 + 1])),
                     float(np.median(y[i0:i1 + 1])),
                     t[i0], t[i1]))

    # --- Tabla de eventos: cada transición clasificada GIRO / AVANCE ---
    print("\nEventos (transición entre mesetas):")
    print(f"  {'#':>5}  {'Δyaw':>9}  {'Δdist':>8}   tipo")
    for k in range(1, len(meds)):
        dang = meds[k][0] - meds[k - 1][0]
        dist = np.hypot(meds[k][1] - meds[k - 1][1], meds[k][2] - meds[k - 1][2])
        if abs(dang) > 30 and dist < 100:
            tipo = f"GIRO {dang:+.1f}°"
        elif dist > 100 and abs(dang) < 30:
            tipo = f"AVANCE {dist:.0f}mm"
        else:
            tipo = f"mixto/ruido (Δyaw={dang:+.1f}, Δd={dist:.0f})"
        print(f"  {k-1:>2}→{k:<2}  {dang:+9.2f}  {dist:8.1f}   {tipo}")

    # --- Resumen de giros cercanos a --expected ---
    if a.expected is not None:
        valid = []
        for k in range(1, len(meds)):
            d = meds[k][0] - meds[k - 1][0]
            if 0.75 * abs(a.expected) <= abs(d) <= 1.25 * abs(a.expected):
                valid.append((abs(d), d - (a.expected if d >= 0 else -a.expected)))
        if valid:
            mags = np.array([v[0] for v in valid])
            ae = np.abs([v[1] for v in valid])
            print(f"\nGiros ≈{abs(a.expected):.0f}°: n={len(valid)}")
            print(f"  Físico real: mediana={np.median(mags):.2f}°  rango=[{mags.min():.2f}, {mags.max():.2f}]")
            print(f"  Error: |medio|={ae.mean():.2f}°  |máx|={ae.max():.2f}°")
            # new_scale = scale_actual * (físico/comandado); robot para cuando
            # yaw_raw*scale==comandado, así que físico>comandado ⇒ subir scale.
            print(f"  yaw_scale sugerido = scale_actual × {np.median(mags) / abs(a.expected):.4f}")
        else:
            print(f"\nNo se hallaron giros cercanos a {a.expected:.0f}°.")

    # --- Resumen de avances ---
    avances = []
    for k in range(1, len(meds)):
        dang = meds[k][0] - meds[k - 1][0]
        dist = np.hypot(meds[k][1] - meds[k - 1][1], meds[k][2] - meds[k - 1][2])
        if dist > 100 and abs(dang) < 30:
            avances.append(dist)
    if avances:
        av = np.array(avances)
        print(f"\nAvances: n={len(av)}  mediana={np.median(av):.1f}mm  rango=[{av.min():.1f}, {av.max():.1f}]")


if __name__ == "__main__":
    main()
