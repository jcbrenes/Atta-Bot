import math
import socket
import json
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import time

@dataclass
class AttaState:
    """Estado de un Atta individual"""
    id: int
    x: float
    y: float
    angle: float
    angular_distance: float
    linear_distance: float
    last_update: float

class SwarmCongregationSystem:
    def __init__(self, base_instance):
        """
        Inicializa el sistema de congregación para el enjambre de Attas
        
        Args:
            base_instance: Instancia de la clase Base del sistema principal
        """
        self.base = base_instance
        self.attas: Dict[int, AttaState] = {}
        self.leader_id: Optional[int] = None
        self.target_radius = 250.0  # mm (radio del círculo de congregación, convertido de 250mm)
        self.min_safe_distance = 100.0  # mm (convertido de 10cm)
        self.congregation_active = False
        
        # Parámetros de control
        self.position_tolerance = 30.0  # mm (convertido de 3cm)
        self.angle_tolerance = 5.0  # grados
        self.max_speed = 200  # velocidad máxima para movimiento
        self.max_distance_per_cycle = 320  # velocidad de aproximación al círculo
        
        # Estado de congregación
        self.congregation_complete = False
        self.last_congregation_check = 0.0
        
    def update_atta_state(self, robot_id: int, x: float, y: float, angle: float):
        """
        Actualiza el estado de un Atta basado en datos del sistema de visión
        
        Args:
            robot_id: ID del robot
            x, y: Posición en mm
            angle: Ángulo en grados
        """
        current_time = time.time()
        
        if robot_id in self.attas:
            # Actualizar estado existente
            prev_state = self.attas[robot_id]
            
            # Calcular distancias recorridas
            linear_dist = math.sqrt((x - prev_state.x)**2 + (y - prev_state.y)**2)
            angular_dist = self._calculate_angular_difference(angle, prev_state.angle)
            
            self.attas[robot_id] = AttaState(
                id=robot_id,
                x=x, y=y, angle=angle,
                angular_distance=prev_state.angular_distance + abs(angular_dist),
                linear_distance=prev_state.linear_distance + linear_dist,
                last_update=current_time
            )
        else:
            # Crear nuevo estado
            self.attas[robot_id] = AttaState(
                id=robot_id,
                x=x, y=y, angle=angle,
                angular_distance=0.0,
                linear_distance=0.0,
                last_update=current_time
            )
    
    def start_congregation(self, leader_id: int) -> bool:
        """
        Inicia el proceso de congregación alrededor del líder especificado
        
        Args:
            leader_id: ID del Atta que será el centro de congregación
            
        Returns:
            bool: True si se pudo iniciar la congregación
        """
        # Convertir a int si es necesario
        leader_id = int(leader_id)
        
        if leader_id not in self.attas:
            print(f"Error: Atta {leader_id} no encontrado para liderar congregación")
            return False
            
        self.leader_id = leader_id
        self.congregation_active = True
        self.congregation_complete = False
        
        print(f"Iniciando congregación alrededor del Atta {leader_id}")
        
        # Enviar comando de parada al líder
        leader_robot = next((robot for robot in self.base.robots.values() if int(robot.id) == leader_id), None)
        if leader_robot:
            self.base.sendInstruction(leader_robot.IP, ['STOP'], True)
            return True
        else:
            print(f"Error: No se encontró el robot líder {leader_id}")
            return False
    
    def process_congregation(self):
        """
        Procesa la lógica de congregación y envía comandos a los Attas
        """
        if not self.congregation_active or self.leader_id is None:
            return
            
        current_time = time.time()
        
        # Verificar cada cierto tiempo si la congregación está completa
        if current_time - self.last_congregation_check > 2.0:
            self._check_congregation_status()
            self.last_congregation_check = current_time
        
        if self.congregation_complete:
            return
            
        leader_state = self.attas.get(self.leader_id)
        if not leader_state:
            return
            
        # Procesar cada Atta (excepto el líder)
        for atta_id, atta_state in self.attas.items():
            if atta_id == self.leader_id:
                continue
                
            # Calcular posición objetivo en el círculo
            target_position = self._calculate_target_position(
                leader_state.x, leader_state.y, atta_state.x, atta_state.y
            )
            
            # Calcular comandos de movimiento
            commands = self._calculate_movement_commands(atta_state, target_position)
            
            # Enviar comandos al Atta
            if commands:
                robot = next((robot for robot in self.base.robots.values() if int(robot.id) == atta_id), None)
                if robot:
                    self.base.sendInstruction(robot.IP, commands, True)
    
    def _calculate_target_position(self, leader_x: float, leader_y: float, 
                                 atta_x: float, atta_y: float) -> Tuple[float, float]:
        """
        Calcula la posición objetivo en el círculo de congregación
        
        Returns:
            Tupla (x, y) de la posición objetivo
        """
        # Vector desde líder hacia el Atta
        dx = atta_x - leader_x
        dy = atta_y - leader_y
        
        # Distancia actual al líder
        current_distance = math.sqrt(dx**2 + dy**2)
        
        if current_distance == 0:
            # Si están en la misma posición, usar ángulo aleatorio
            angle = math.radians(hash(time.time()) % 360)
            target_x = leader_x + self.target_radius * math.cos(angle)
            target_y = leader_y + self.target_radius * math.sin(angle)
        else:
            # Normalizar el vector y escalarlo al radio objetivo
            scale_factor = self.target_radius / current_distance
            target_x = leader_x + dx * scale_factor
            target_y = leader_y + dy * scale_factor
        
        return target_x, target_y
    
    def _calculate_movement_commands(self, atta_state: AttaState, 
                                   target_pos: Tuple[float, float]) -> List[str]:
        """
        Calcula los comandos de movimiento necesarios para alcanzar la posición objetivo
        
        Returns:
            Lista de comandos para enviar al Atta
        """
        target_x, target_y = target_pos
        
        # Vector hacia el objetivo
        dx = target_x - atta_state.x
        dy = target_y - atta_state.y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        # Si ya está cerca del objetivo, no hacer nada
        if distance_to_target < self.position_tolerance:
            return []
        
        # Calcular ángulo hacia el objetivo
        target_angle = math.degrees(math.atan2(dy, dx)) % 360
        
        # Diferencia angular que necesita girar
        angle_diff = self._calculate_angular_difference(target_angle, atta_state.angle)
        
        commands = []
        
        # Si necesita girar significativamente, primero girar
        if abs(angle_diff) > self.angle_tolerance:
            commands.append(f'TURN|{int(angle_diff)}')
        
        # Calcular distancia de movimiento (limitada por velocidad)
        move_distance = min(distance_to_target, self.max_distance_per_cycle)
        
        if move_distance > self.position_tolerance:
            commands.append(f'MOVE|{int(move_distance)}')
        
        return commands
    
    def _calculate_angular_difference(self, target_angle: float, current_angle: float) -> float:
        """
        Calcula la diferencia angular más corta entre dos ángulos
        
        Returns:
            Diferencia angular en grados (-180 a 180)
        """
        diff = target_angle - current_angle
        
        # Normalizar a rango [-180, 180]
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
            
        return diff
    
    def _check_congregation_status(self):
        """
        Verifica si la congregación está completa
        """
        if self.leader_id is None:
            return
            
        leader_state = self.attas.get(self.leader_id)
        if not leader_state:
            return
            
        attas_in_position = 0
        total_attas = len(self.attas) - 1  # Excluir líder
        
        for atta_id, atta_state in self.attas.items():
            if atta_id == self.leader_id:
                continue
                
            # Distancia al líder
            distance_to_leader = math.sqrt(
                (atta_state.x - leader_state.x)**2 + 
                (atta_state.y - leader_state.y)**2
            )
            
            # Verificar si está en el radio objetivo
            if abs(distance_to_leader - self.target_radius) < self.position_tolerance:
                attas_in_position += 1
        
        # Congregación completa si todos están en posición
        if attas_in_position >= total_attas * 0.8:  # 80% de tolerancia
            if not self.congregation_complete:
                print(f"¡Congregación completa! {attas_in_position}/{total_attas} Attas en posición")
                self.congregation_complete = True
    
    def stop_congregation(self):
        """
        Detiene el proceso de congregación
        """
        self.congregation_active = False
        self.leader_id = None
        self.congregation_complete = False
        print("Congregación detenida")
    
    def get_congregation_status(self) -> Dict:
        """
        Obtiene el estado actual de la congregación
        
        Returns:
            Diccionario con información del estado
        """
        if not self.congregation_active:
            return {"active": False}
            
        leader_state = self.attas.get(self.leader_id) if self.leader_id else None
        
        status = {
            "active": self.congregation_active,
            "leader_id": self.leader_id,
            "complete": self.congregation_complete,
            "target_radius": self.target_radius,
            "attas_count": len(self.attas),
            "leader_position": (leader_state.x, leader_state.y) if leader_state else None
        }
        
        # Calcular distancias de cada Atta al líder
        if leader_state:
            distances = {}
            for atta_id, atta_state in self.attas.items():
                if atta_id != self.leader_id:
                    dist = math.sqrt(
                        (atta_state.x - leader_state.x)**2 + 
                        (atta_state.y - leader_state.y)**2
                    )
                    distances[atta_id] = round(dist, 1)
            status["distances_to_leader"] = distances
        
        return status

# Funciones de integración con el sistema principal

def integrate_congregation_system(base_instance):
    """
    Integra el sistema de congregación con el sistema principal
    
    Args:
        base_instance: Instancia de la clase Base
    """
    # Crear instancia del sistema de congregación
    congregation_system = SwarmCongregationSystem(base_instance)
    
    # Agregar referencia al sistema base
    base_instance.congregation_system = congregation_system
    
    # Modificar el método sendPositions para actualizar estados
    original_send_positions = base_instance.sendPositions
    
    def enhanced_send_positions(resultsFrame, timeLog):
        # Actualizar estados de congregación antes de enviar posiciones
        for robot in base_instance.robots.values():
            if hasattr(robot, 'previousPose') and robot.previousPose != (-1, -1, -1):
                x, y, angle = robot.previousPose
                # Asegurar que robot.id sea int
                robot_id = int(robot.id)
                congregation_system.update_atta_state(robot_id, x, y, angle)
        
        # Procesar congregación si está activa
        congregation_system.process_congregation()
        
        # Llamar al método original
        original_send_positions(resultsFrame, timeLog)
    
    # Reemplazar el método
    base_instance.sendPositions = enhanced_send_positions
    
    # Agregar comandos de congregación al sistema de entrada
    original_input_instruction = base_instance.inputInstruction.__func__
    
    def enhanced_input_instruction(self):
        """
        Versión mejorada que incluye comandos de congregación
        """
        print("\nComandos de congregación disponibles:")
        print("  CONGREGATION.START.<leader_id> - Iniciar congregación")
        print("  CONGREGATION.STOP - Detener congregación")
        print("  CONGREGATION.STATUS - Ver estado de congregación")
        print("  Ejemplo: CONGREGATION.START.4\n")
        
        while True:
            instructionRaw = input('').strip()
            if instructionRaw == 'BREAK':
                break
            
            # Procesar comandos de congregación
            if instructionRaw.startswith('CONGREGATION.'):
                congregation_command = instructionRaw[13:]  # Remover 'CONGREGATION.'
                
                if congregation_command.startswith('START.'):
                    try:
                        leader_id = int(congregation_command[6:])
                        if congregation_system.start_congregation(leader_id):
                            print(f"Congregación iniciada con líder: {leader_id}")
                        else:
                            print(f"Error: No se pudo iniciar congregación con líder {leader_id}")
                    except ValueError:
                        print("Error: ID de líder debe ser un número")
                
                elif congregation_command == 'STOP':
                    congregation_system.stop_congregation()
                
                elif congregation_command == 'STATUS':
                    status = congregation_system.get_congregation_status()
                    print("Estado de congregación:")
                    for key, value in status.items():
                        print(f"  {key}: {value}")
                
                else:
                    print("Comando de congregación no reconocido")
                
                continue
            
            # Procesar comandos normales
            try:
                robotId, instruction = map(str.strip, instructionRaw.split('.', 1))
            except ValueError:
                print("Formato inválido. Use 'robotId.instrucción' o comandos CONGREGATION.*")
                continue

            if robotId == 'BROADCAST':
                self.sendInstructionBroadcast([instruction])
            elif robotId.isdigit():
                # Manejar IDs numéricos
                robotId_int = int(robotId)
                if robotId_int in self.robots:
                    robotIP = self.robots[robotId_int].IP
                    self.sendInstruction(robotIP, [instruction], True)
                else:
                    print(f"Robot ID '{robotId}' no encontrado.")
            elif robotId in self.robots:
                # Manejar IDs como strings
                robotIP = self.robots[robotId].IP
                self.sendInstruction(robotIP, [instruction], True)
            else:
                print(f"Robot ID '{robotId}' no encontrado.")
        
        self.threadInputAlive = False
    
    # Aplicar el método mejorado usando binding manual y decorador de threading
    import types
    from threading import Thread
    
    def threaded_enhanced_input(self):
        thread = Thread(target=enhanced_input_instruction, args=(self,), daemon=True)
        thread.start()
        return thread
    
    base_instance.inputInstruction = types.MethodType(threaded_enhanced_input, base_instance)
    
    return congregation_system

# Ejemplo de uso:
"""
Comandos disponibles durante la ejecución:
- CONGREGATION.START.4 (inicia congregación alrededor del Atta 4)
- CONGREGATION.STOP (detiene congregación)
- CONGREGATION.STATUS (muestra estado actual)
"""