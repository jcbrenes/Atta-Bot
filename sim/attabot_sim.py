"""
AttaBot 2D Simulator
====================
Replica la lógica de ReactiveNavStep() del firmware en Python puro.
Permite probar GT, congregación y comportamientos de enjambre sin lab.

Uso:
    python attabot_sim.py --scenario gt
    python attabot_sim.py --scenario congregation
    python attabot_sim.py --scenario obstacle

El eje Y apunta hacia abajo (igual que la cámara ArUco).
Ángulo 0° = derecha (+X), aumenta en sentido horario (igual que el firmware).
"""

import math
import random
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


# ── Constantes del robot (mismos valores que el firmware) ────────────────────
CENTER_TO_WHEEL    = 41.5   # mm — igual que centerToWheelDistance en .ino
ARRIVAL_THRESHOLD  = 50.0   # mm
SEGMENT_DISTANCE   = 250.0  # mm
AVOID_SEGMENT      = 120.0  # mm durante evasión
AVOID_FRONT_ANGLE  = 90.0   # grados cuando hay obstáculo frontal
AVOID_SIDE_ANGLE   = 35.0   # grados cuando hay obstáculo lateral
IR_RANGE           = 200.0  # mm — distancia detección IR simulada
ROBOT_RADIUS       = 75.0   # mm — radio del cuerpo del robot


# ── Ruido del sistema (calibrado en lab) ─────────────────────────────────────
ARUCO_POS_SIGMA    = 30.0   # mm — ruido de posición ArUco
ARUCO_ANGLE_SIGMA  = 2.0    # grados — ruido de ángulo ArUco
MOTOR_TURN_FACTOR  = 1.0    # 1.0 = calibrado perfecto; 0.77 = robot 3 sin IMU
MOTOR_DRIFT_SIGMA  = 0.02   # fracción de la distancia (2% de error en MOVE)


# ── Utilidades de ángulo ─────────────────────────────────────────────────────
def normalize_angle(a: float) -> float:
    """Normaliza ángulo a [-180, 180]."""
    while a >  180: a -= 360
    while a < -180: a += 360
    return a


# ── Modelo de robot ──────────────────────────────────────────────────────────
@dataclass
class SimRobot:
    x: float
    y: float
    angle: float          # grados, 0=derecha, CW positivo
    robot_id: int = 0
    color: str = 'blue'

    # Modelo de motor: factor < 1 simula subgiro (ej. robot 3 sin IMU)
    turn_factor: float = MOTOR_TURN_FACTOR

    # Historial para visualización
    trajectory: List[Tuple[float, float]] = field(default_factory=list)

    def __post_init__(self):
        self.trajectory.append((self.x, self.y))

    def turn(self, arc_distance: float):
        """Ejecuta un giro. arc_distance = radianes(angleDiff) * centerToWheelDistance."""
        actual_deg = (arc_distance / CENTER_TO_WHEEL) * (180 / math.pi)
        actual_deg *= self.turn_factor
        actual_deg += random.gauss(0, 0.5)   # ruido pequeño
        self.angle = (self.angle + actual_deg) % 360

    def move(self, distance: float, world: 'SimWorld' = None):
        """
        Avanza en sub-pasos de 20mm para registrar trayectoria real
        y detectar colisiones durante el movimiento.
        Devuelve True si completó el movimiento, False si colisionó.
        """
        actual_dist = distance * (1 + random.gauss(0, MOTOR_DRIFT_SIGMA))
        rad = math.radians(self.angle)
        step = 20.0  # sub-pasos de 20mm
        traveled = 0.0
        while traveled < actual_dist:
            d = min(step, actual_dist - traveled)
            self.x += math.cos(rad) * d
            self.y += math.sin(rad) * d
            traveled += d
            self.trajectory.append((self.x, self.y))
            # Colisión con obstáculos durante el movimiento
            if world and world.collides(self):
                # Retroceder al último punto válido
                self.x -= math.cos(rad) * d
                self.y -= math.sin(rad) * d
                self.trajectory[-1] = (self.x, self.y)
                return False
        return True

    def get_pose_noisy(self) -> Tuple[float, float, float]:
        """Simula lo que devuelve ArUco: posición real + ruido gaussiano."""
        nx = self.x + random.gauss(0, ARUCO_POS_SIGMA)
        ny = self.y + random.gauss(0, ARUCO_POS_SIGMA)
        na = self.angle + random.gauss(0, ARUCO_ANGLE_SIGMA)
        return nx, ny, na % 360


# ── Modelo de mundo ──────────────────────────────────────────────────────────
@dataclass
class Obstacle:
    x: float
    y: float
    radius: float = 80.0  # mm — radio de un Atta (~80mm)


class SimWorld:
    def __init__(self, width=1800, height=1200):
        self.width    = width
        self.height   = height
        self.robots: List[SimRobot]   = []
        self.obstacles: List[Obstacle] = []

    def collides(self, robot: SimRobot) -> bool:
        """True si el cuerpo del robot toca un obstáculo ESTÁTICO."""
        for o in self.obstacles:
            dx = robot.x - o.x; dy = robot.y - o.y
            if math.sqrt(dx*dx + dy*dy) < o.radius + ROBOT_RADIUS:
                return True
        return False

    def check_ir(self, robot: SimRobot) -> dict:
        """
        Simula los sensores IR del robot usando distancia real al borde del obstáculo.
        Detecta si el borde (superficie) del obstáculo está dentro del rango IR
        y en el ángulo del sensor (±45°).
        """
        # Solo obstáculos físicos — otros robots se detectan por ArUco, no por IR
        targets = [(o.x, o.y, o.radius) for o in self.obstacles]

        sensor_angles = {
            'front': robot.angle,
            'right': robot.angle - 30,
            'left':  robot.angle + 30,
        }
        result = {'front': False, 'right': False, 'left': False}

        for (tx, ty, tr) in targets:
            dx = tx - robot.x; dy = ty - robot.y
            dist_to_center = math.sqrt(dx*dx + dy*dy)
            dist_to_edge   = max(0.0, dist_to_center - tr - ROBOT_RADIUS)
            if dist_to_edge > IR_RANGE:
                continue
            # Ángulo hacia el obstáculo
            angle_to_obs = math.degrees(math.atan2(dy, dx)) % 360
            for key, sang in sensor_angles.items():
                sang = sang % 360
                diff = abs(((angle_to_obs - sang) + 180) % 360 - 180)
                if diff < 45:
                    result[key] = True

        return result


# ── ReactiveNav — port directo del firmware ──────────────────────────────────
class ReactiveNav:
    """
    Port Python de ReactiveNavStep() en AttaBot.ino.
    Misma lógica, mismos parámetros, mismo comportamiento esperado.
    """
    def __init__(self):
        self.goal_x: float = 0
        self.goal_y: float = 0
        self.is_active: bool  = False
        self.steps:    int    = 0

    def start(self, gx: float, gy: float):
        self.goal_x    = gx
        self.goal_y    = gy
        self.is_active = True
        self.steps     = 0

    def has_reached(self, x: float, y: float) -> bool:
        dx = self.goal_x - x; dy = self.goal_y - y
        return math.sqrt(dx*dx + dy*dy) < ARRIVAL_THRESHOLD

    def step(self, robot: SimRobot, ir: dict) -> Optional[Tuple[float, float]]:
        """
        Calcula (arc_distance_turn, move_distance) para el próximo paso.
        Devuelve None si el robot llegó.
        Misma lógica que ReactiveNavStep() en el firmware.
        """
        px, py, pangle = robot.get_pose_noisy()

        if self.has_reached(px, py):
            self.is_active = False
            return None

        dx = self.goal_x - px
        dy = self.goal_y - py
        dist = math.sqrt(dx*dx + dy*dy)

        goal_angle = math.degrees(math.atan2(dy, dx)) % 360

        # ── Capa reactiva ────────────────────────────────────────────────────
        front   = ir['front']
        right   = ir['right']
        left    = ir['left']
        bias    = 0.0
        avoiding = False

        if front:
            # Elegir lado hacia el objetivo
            rel_goal = normalize_angle(goal_angle - pangle)
            bias     = -AVOID_FRONT_ANGLE if rel_goal >= 0 else AVOID_FRONT_ANGLE
            avoiding = True
        elif right:
            bias     = AVOID_SIDE_ANGLE    # lean izquierda
            avoiding = True
        elif left:
            bias     = -AVOID_SIDE_ANGLE   # lean derecha
            avoiding = True

        final_angle = (goal_angle + bias) % 360
        angle_diff  = normalize_angle(final_angle - pangle)

        seg = AVOID_SEGMENT if avoiding else min(dist * 0.9, SEGMENT_DISTANCE)
        seg = max(10.0, seg)

        arc = math.radians(angle_diff) * CENTER_TO_WHEEL
        self.steps += 1
        return arc, seg


# ── Bucle de simulación ──────────────────────────────────────────────────────
def run_simulation(robot: SimRobot, nav: ReactiveNav, world: SimWorld,
                   max_steps: int = 100) -> dict:
    """
    Ejecuta la simulación.
    El IR se verifica al inicio de cada paso Y durante el MOVE (sub-pasos de 20mm),
    igual que el firmware donde el sensor IR dispara por interrupción.
    Si durante el MOVE se detecta un obstáculo, el paso se interrumpe y
    se recalcula la navegación desde la posición actual.
    """
    arrived = False
    for _ in range(max_steps):
        ir = world.check_ir(robot)
        result = nav.step(robot, ir)
        if result is None:
            arrived = True
            break
        arc, seg = result

        if abs(arc) > math.radians(5) * CENTER_TO_WHEEL:
            robot.turn(arc)

        # MOVE con re-evaluación IR cada sub-paso (como interrupción en firmware)
        actual_dist = seg * (1 + random.gauss(0, MOTOR_DRIFT_SIGMA))
        rad = math.radians(robot.angle)
        sub = 20.0
        traveled = 0.0
        interrupted = False
        while traveled < actual_dist:
            d = min(sub, actual_dist - traveled)
            robot.x += math.cos(rad) * d
            robot.y += math.sin(rad) * d
            traveled += d
            robot.trajectory.append((robot.x, robot.y))

            if world.collides(robot):
                robot.x -= math.cos(rad) * d
                robot.y -= math.sin(rad) * d
                robot.trajectory[-1] = (robot.x, robot.y)
                interrupted = True
                break

            # Chequeo IR continuo durante el movimiento
            mid_ir = world.check_ir(robot)
            if any(mid_ir.values()) and not any(ir.values()):
                interrupted = True
                break

        if interrupted:
            nav.steps += 1  # cuenta como paso

    dx = nav.goal_x - robot.x
    dy = nav.goal_y - robot.y
    final_dist = math.sqrt(dx*dx + dy*dy)

    return {
        'arrived':    arrived,
        'steps':      nav.steps,
        'final_dist': final_dist,
        'path_len':   sum(
            math.sqrt((robot.trajectory[i][0]-robot.trajectory[i-1][0])**2 +
                      (robot.trajectory[i][1]-robot.trajectory[i-1][1])**2)
            for i in range(1, len(robot.trajectory))
        ),
    }


# ── Visualización ────────────────────────────────────────────────────────────
def plot_scenario(world: SimWorld, nav_goals: list, title: str, metrics: list):
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.set_xlim(0, world.width)
    ax.set_ylim(world.height, 0)   # Y invertido = igual que cámara
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=13)
    ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)')
    ax.grid(True, alpha=0.3)

    # Obstáculos — relleno + borde sólido para ver claramente el límite
    for obs in world.obstacles:
        ax.add_patch(plt.Circle((obs.x, obs.y), obs.radius,
                                color='gray', alpha=0.4))
        ax.add_patch(plt.Circle((obs.x, obs.y), obs.radius,
                                color='black', fill=False, linewidth=1.5))
        # Zona de detección IR (radio + IR_RANGE)
        ax.add_patch(plt.Circle((obs.x, obs.y), obs.radius + IR_RANGE,
                                color='gray', fill=False,
                                linestyle=':', linewidth=0.8, alpha=0.4))

    # Objetivos
    for (gx, gy) in nav_goals:
        ax.plot(gx, gy, 'r*', markersize=15, zorder=5)
        circle = plt.Circle((gx, gy), ARRIVAL_THRESHOLD,
                             color='red', fill=False, linestyle='--', alpha=0.4)
        ax.add_patch(circle)

    # Trayectorias
    colors = ['blue', 'green', 'orange', 'purple']
    for i, robot in enumerate(world.robots):
        if len(robot.trajectory) < 2:
            continue
        xs, ys = zip(*robot.trajectory)
        c = colors[i % len(colors)]
        ax.plot(xs, ys, '-', color=c, linewidth=1.5, alpha=0.7)
        ax.plot(xs[0], ys[0], 'o', color=c, markersize=8)   # inicio
        ax.plot(xs[-1], ys[-1], 's', color=c, markersize=8)  # fin
        # Heading final
        rad = math.radians(robot.angle)
        ax.annotate('', xy=(robot.x + math.cos(rad)*60, robot.y + math.sin(rad)*60),
                    xytext=(robot.x, robot.y),
                    arrowprops=dict(arrowstyle='->', color=c, lw=2))

    # Métricas en el plot
    if metrics:
        info = '\n'.join([
            f"Robot {m['id']}: {'✓' if m['arrived'] else '✗'} "
            f"{m['steps']} pasos | dist final {m['final_dist']:.0f}mm | "
            f"eficiencia {m.get('efficiency', 0):.2f}"
            for m in metrics
        ])
        ax.text(0.02, 0.02, info, transform=ax.transAxes, fontsize=8,
                verticalalignment='bottom',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    plt.tight_layout()
    plt.savefig(f"sim_output_{title.replace(' ','_').lower()}.png", dpi=150)
    print(f"Guardado: sim_output_{title.replace(' ','_').lower()}.png")
    plt.show()


# ── Escenarios ───────────────────────────────────────────────────────────────
def scenario_gt_basic():
    """GT punto a punto sin obstáculos."""
    world  = SimWorld()
    robot  = SimRobot(x=200, y=600, angle=90, robot_id=1, color='blue')
    world.robots.append(robot)

    nav = ReactiveNav()
    nav.start(1400, 600)
    metrics_raw = run_simulation(robot, nav, world)

    straight = math.sqrt((1400-200)**2 + (600-600)**2)
    m = {**metrics_raw, 'id': 1,
         'efficiency': straight / metrics_raw['path_len'] if metrics_raw['path_len'] > 0 else 0}
    print(f"\n── GT básico ──\n{m}")
    plot_scenario(world, [(1400, 600)], 'GT básico sin obstáculos', [m])


def scenario_gt_obstacle():
    """GT con obstáculo desviado de la trayectoria directa."""
    world = SimWorld()
    # Robot va de (200,900) a (1400,300) — línea diagonal
    robot = SimRobot(x=200, y=900, angle=340, robot_id=1)
    world.robots.append(robot)
    # Obstáculo cerca del centro del camino pero no exactamente en él
    world.obstacles.append(Obstacle(x=750, y=570, radius=100))
    world.obstacles.append(Obstacle(x=900, y=650, radius=80))

    nav = ReactiveNav()
    nav.start(1500, 600)
    metrics_raw = run_simulation(robot, nav, world)

    straight = math.sqrt((1500-150)**2)
    m = {**metrics_raw, 'id': 1,
         'efficiency': straight / metrics_raw['path_len'] if metrics_raw['path_len'] > 0 else 0}
    print(f"\n── GT con obstáculo ──\n{m}")
    plot_scenario(world, [(1500, 600)], 'GT con barrera', [m])


def scenario_congregation():
    """3 robots convergen hacia el robot 1 (líder)."""
    world = SimWorld()
    colors = ['blue', 'green', 'orange']
    starts = [(200, 200), (1600, 300), (900, 1050)]
    leader_pos = (900, 600)

    all_metrics = []
    for i, (sx, sy) in enumerate(starts):
        robot = SimRobot(x=sx, y=sy, angle=random.uniform(0, 360),
                         robot_id=i+1, color=colors[i])
        world.robots.append(robot)

    for i, robot in enumerate(world.robots):
        nav = ReactiveNav()
        nav.start(*leader_pos)
        metrics_raw = run_simulation(robot, nav, world, max_steps=150)
        straight = math.sqrt((leader_pos[0]-starts[i][0])**2 +
                             (leader_pos[1]-starts[i][1])**2)
        m = {**metrics_raw, 'id': i+1,
             'efficiency': straight / metrics_raw['path_len'] if metrics_raw['path_len'] > 0 else 0}
        all_metrics.append(m)
        print(f"Robot {i+1}: {m}")

    # Líder como estrella grande
    ax_goal = [leader_pos]
    plot_scenario(world, ax_goal, 'Congregación 3 robots', all_metrics)


# ── Main ─────────────────────────────────────────────────────────────────────
SCENARIOS = {
    'gt':           scenario_gt_basic,
    'obstacle':     scenario_gt_obstacle,
    'congregation': scenario_congregation,
}

def scenario_turn_factor_comparison():
    """Compara robot calibrado vs robot 3 con subgiro 23% (sin IMU)."""
    import copy

    configs = [
        (1.00, 'Robot calibrado (factor=1.0)'),
        (0.77, 'Robot 3 sin IMU (factor=0.77)'),
    ]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Impacto del subgiro de motor — calibrado vs robot 3', fontsize=13)

    for ax, (factor, label) in zip(axes, configs):
        world = SimWorld()
        world.obstacles.append(Obstacle(x=800, y=600, radius=100))
        robot = SimRobot(x=150, y=600, angle=10, robot_id=1,
                         turn_factor=factor, color='blue')
        world.robots.append(robot)
        nav = ReactiveNav()
        nav.start(1500, 600)
        m = run_simulation(robot, nav, world)
        straight = math.sqrt((1500-150)**2)
        efficiency = straight / m['path_len'] if m['path_len'] > 0 else 0

        ax.set_xlim(0, 1800); ax.set_ylim(1200, 0)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_title(label)
        ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)')

        for obs in world.obstacles:
            ax.add_patch(plt.Circle((obs.x, obs.y), obs.radius, color='gray', alpha=0.5))
        ax.plot(1500, 600, 'r*', markersize=15)
        ax.add_patch(plt.Circle((1500, 600), ARRIVAL_THRESHOLD,
                                color='red', fill=False, linestyle='--', alpha=0.4))
        if robot.trajectory:
            xs, ys = zip(*robot.trajectory)
            ax.plot(xs, ys, 'b-', linewidth=2, alpha=0.8)
            ax.plot(xs[0], ys[0], 'go', markersize=10, label='Inicio')
            ax.plot(xs[-1], ys[-1], 'bs', markersize=10, label='Fin')

        info = (f"{'✓ Llegó' if m['arrived'] else '✗ No llegó'} | "
                f"{m['steps']} pasos\nEficiencia: {efficiency:.3f} | "
                f"Error: {m['final_dist']:.0f}mm")
        ax.text(0.02, 0.02, info, transform=ax.transAxes, fontsize=9,
                verticalalignment='bottom',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        ax.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig('sim_output_turn_factor_comparison.png', dpi=150)
    print("Guardado: sim_output_turn_factor_comparison.png")
    plt.show()


def scenario_angle_offset():
    """Muestra cómo el marker rotado destruye la navegación."""
    offsets = [0, 45, 90, 125]
    labels  = ['0° (correcto)', '45°', '90° (robot 3 antes)', '125° (lab)']
    colors  = ['blue', 'green', 'orange', 'red']

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Impacto del offset de ángulo ArUco en navegación', fontsize=13)
    axes = axes.flatten()

    original_get_pose = SimRobot.get_pose_noisy

    for ax, offset, label, color in zip(axes, offsets, labels, colors):
        def make_noisy(off):
            def get_pose_offset(self):
                x, y, a = original_get_pose(self)
                return x, y, (a + off) % 360
            return get_pose_offset
        SimRobot.get_pose_noisy = make_noisy(offset)

        world = SimWorld()
        world.obstacles.append(Obstacle(x=800, y=600, radius=100))
        robot = SimRobot(x=150, y=600, angle=10, robot_id=1, color=color)
        world.robots.append(robot)
        nav = ReactiveNav()
        nav.start(1500, 600)
        m = run_simulation(robot, nav, world, max_steps=60)
        straight = math.sqrt((1500-150)**2)
        efficiency = straight / m['path_len'] if m['path_len'] > 0 else 0

        ax.set_xlim(0, 1800); ax.set_ylim(1200, 0)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
        ax.set_title(f'Marker offset = {label}')
        ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)')

        for obs in world.obstacles:
            ax.add_patch(plt.Circle((obs.x, obs.y), obs.radius, color='gray', alpha=0.5))
        ax.plot(1500, 600, 'r*', markersize=15)
        ax.add_patch(plt.Circle((1500, 600), ARRIVAL_THRESHOLD,
                                color='red', fill=False, linestyle='--', alpha=0.4))
        if robot.trajectory:
            xs, ys = zip(*robot.trajectory)
            ax.plot(xs, ys, color=color, linewidth=2, alpha=0.8)
            ax.plot(xs[0], ys[0], 'o', color=color, markersize=10)
            ax.plot(xs[-1], ys[-1], 's', color=color, markersize=10)

        status = '✓ Llegó' if m['arrived'] else '✗ No llegó'
        info = f"{status} | {m['steps']} pasos\nEficiencia: {efficiency:.3f}"
        ax.text(0.02, 0.02, info, transform=ax.transAxes, fontsize=9,
                verticalalignment='bottom',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    SimRobot.get_pose_noisy = original_get_pose
    plt.tight_layout()
    plt.savefig('sim_output_angle_offset.png', dpi=150)
    print("Guardado: sim_output_angle_offset.png")
    plt.show()


SCENARIOS = {
    'gt':           scenario_gt_basic,
    'obstacle':     scenario_gt_obstacle,
    'congregation': scenario_congregation,
    'turn_factor':  scenario_turn_factor_comparison,
    'angle_offset': scenario_angle_offset,
}

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='AttaBot 2D Simulator')
    parser.add_argument('--scenario', choices=list(SCENARIOS.keys()) + ['all'],
                        default='all', help='Escenario a simular (default: all)')
    parser.add_argument('--seed', type=int, default=42,
                        help='Semilla random para reproducibilidad')
    parser.add_argument('--no-show', action='store_true',
                        help='No mostrar ventana, solo guardar PNG')
    args = parser.parse_args()
    if args.no_show:
        import matplotlib
        matplotlib.use('Agg')

    random.seed(args.seed)
    if args.scenario == 'all':
        for name, fn in SCENARIOS.items():
            print(f"\n{'='*40}\nEscenario: {name}\n{'='*40}")
            random.seed(args.seed)
            fn()
    else:
        SCENARIOS[args.scenario]()
