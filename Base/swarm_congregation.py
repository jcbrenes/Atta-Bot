import socket
import threading
import time

class CongregationSystem:
    """
    Sistema de congregación para robots de enjambre que integra con el sistema de visión existente.
    
    Esta clase maneja la comunicación UDP con los robots para coordinar movimientos de congregación,
    donde un robot líder se mantiene estático y los demás robots se mueven hacia su posición.
    """
    
    def __init__(self, base):
        """
        Inicializa el sistema de congregación.
        
        Args:
            base: Instancia de la clase Base que contiene la configuración del sistema
        """
        self.base = base
        self.congregation_active = False
        self.current_leader_id = None
        self.pending_position_requests = set()
        self.congregation_thread = None
        
        # Mapeo de IPs a IDs de robots - se actualiza automáticamente
        self.ip_to_robot_id = {}
        
        print("Sistema de congregación inicializado")
    
    def update_ip_mapping(self):
        """
        Actualiza el mapeo de IPs a IDs de robots basándose en los robots detectados.
        """
        self.ip_to_robot_id = {robot.IP: robot.id for robot in self.base.robots.values()}
        print(f"Mapeo IP actualizado: {self.ip_to_robot_id}")
    
    def start_congregation(self, leader_id):
        """
        Inicia el proceso de congregación con el líder especificado.
        
        Args:
            leader_id (str): ID del robot que actuará como líder
        """
        if leader_id not in self.base.robots:
            print(f"Error: Robot {leader_id} no encontrado en el sistema")
            return False
        
        self.congregation_active = True
        self.current_leader_id = leader_id
        self.pending_position_requests.clear()
        
        # Actualizar mapeo de IPs
        self.update_ip_mapping()
        
        print(f"Iniciando congregación con líder: {leader_id}")
        
        # Enviar comando de congregación broadcast
        instruction = f"CONGREGATION|{leader_id}"
        self.base.sendInstructionBroadcast([instruction])
        
        print(f"Comando enviado: BROADCAST.{instruction}")
        return True
    
    def handle_position_request(self, sender_ip):
        """
        Maneja solicitudes de posición de los robots.
        
        Args:
            sender_ip (str): IP del robot que solicita su posición
        """
        robot_id = self.ip_to_robot_id.get(sender_ip)
        
        if not robot_id:
            print(f"IP no reconocida en solicitud de posición: {sender_ip}")
            return
        
        if robot_id not in self.base.robots:
            print(f"Robot {robot_id} no encontrado en sistema")
            return
        
        # Obtener posición actual del robot desde el sistema de visión
        robot = self.base.robots[robot_id]
        current_pose = robot.getPose()
        
        if current_pose == (-1, -1, -1):
            print(f"No se pudo obtener posición para robot {robot_id}")
            return
        
        x, y, angle = current_pose
        
        # Enviar respuesta con posición
        response = f"POSITION_RESPONSE|{x}|{y}|{angle}"
        self.base.sendInstruction(sender_ip, [response], False)
        
        print(f"Posición enviada a robot {robot_id}: {x}, {y}, {angle}")
        
        # Marcar como atendido
        if robot_id in self.pending_position_requests:
            self.pending_position_requests.remove(robot_id)
    
    def handle_congregation_message(self, message, sender_ip):
        """
        Procesa mensajes relacionados con congregación.
        
        Args:
            message (str): Mensaje recibido
            sender_ip (str): IP del remitente
            
        Returns:
            bool: True si el mensaje fue procesado, False si no es un mensaje de congregación
        """
        if not self.congregation_active:
            return False
        
        parts = message.split('|')
        command = parts[0]
        
        if command == "REQUEST_POSITION":
            self.handle_position_request(sender_ip)
            return True
            
        elif command == "COORDS_RECEIVED":
            robot_id = self.ip_to_robot_id.get(sender_ip, "unknown")
            print(f"Robot {robot_id} confirmó recepción de coordenadas")
            return True
            
        elif command == "ALL_MOVEMENTS_COMPLETE":
            robot_id = self.ip_to_robot_id.get(sender_ip, "unknown")
            print(f"Robot {robot_id} completó movimientos de congregación")
            return True
            
        elif command == "LEADER_POSITION":
            # Los robots intercambian información de líder - solo logging
            leader_id = parts[1] if len(parts) > 1 else "unknown"
            robot_id = self.ip_to_robot_id.get(sender_ip, "unknown")
            print(f"Robot {robot_id} recibió posición del líder {leader_id}")
            return True
        
        return False
    
    def stop_congregation(self):
        """
        Detiene el proceso de congregación activo.
        """
        self.congregation_active = False
        self.current_leader_id = None
        self.pending_position_requests.clear()
        print("Congregación detenida")
    
    def get_congregation_status(self):
        """
        Obtiene el estado actual de la congregación.
        
        Returns:
            dict: Estado de la congregación
        """
        return {
            'active': self.congregation_active,
            'leader_id': self.current_leader_id,
            'pending_requests': len(self.pending_position_requests),
            'total_robots': len(self.base.robots)
        }


def integrate_congregation_system(base):
    """
    Integra el sistema de congregación con la clase Base existente.
    
    Args:
        base: Instancia de la clase Base
        
    Returns:
        CongregationSystem: Instancia del sistema de congregación
    """
    congregation_system = CongregationSystem(base)
    
    # Modificar el método readUdpConnection de la clase Base
    original_readUdpConnection = base.readUdpConnection
    
    def enhanced_readUdpConnection():
        """
        Versión mejorada de readUdpConnection que incluye manejo de congregación.
        """
        base.createConcoleLog()
        base.sock.settimeout(None)
        
        while True:
            try:
                data, addr = base.sock.recvfrom(1024)
                ip = addr[0]
                
                if ip != base.baseIP:
                    message = data.decode()
                    timeLog = round(time.time() - base.startTime, 1)
                    
                    # Intentar procesar mensaje de congregación primero
                    if congregation_system.handle_congregation_message(message, ip):
                        # Mensaje de congregación procesado
                        continue
                    
                    # Procesamiento normal para otros mensajes
                    robot_info = next(
                        ((robot.name, robot.id) for robot in base.robots.values() if robot.IP == ip), 
                        (ip, "unknown")
                    )
                    name, robot_id = robot_info
                    base.addConcoleLog(timeLog, robot_id, name, message)
                    
                    if message.split('|')[0] == 'CHECK_OBSTACLE':
                        continue
                        
                    print(f"Mensaje de {name}: {message}")
                    
            except Exception as e:
                print(f"Error en comunicación UDP: {e}")
                break
    
    # Reemplazar el método original con la versión mejorada
    base.readUdpConnection = enhanced_readUdpConnection
    
    # Modificar el método inputInstruction para incluir comandos de congregación
    original_inputInstruction = base.inputInstruction
    
    def enhanced_inputInstruction():
        """
        Versión mejorada de inputInstruction que incluye comandos de congregación.
        """
        print("Comandos disponibles:")
        print("  - robotId.instrucción (ej: 1.MOVE|100)")
        print("  - BROADCAST.instrucción")
        print("  - CONGREGATION.START|leader_id (ej: CONGREGATION.START|4)")
        print("  - CONGREGATION.STOP")
        print("  - CONGREGATION.STATUS")
        print("  - BREAK (para salir)")
        
        while True:
            instructionRaw = input('>> ').strip()
            
            if instructionRaw == 'BREAK':
                break
            
            # Comandos de congregación
            if instructionRaw.startswith('CONGREGATION.'):
                command_part = instructionRaw[13:]  # Remover 'CONGREGATION.'
                
                if command_part.startswith('START|'):
                    leader_id = command_part[6:]  # Remover 'START|'
                    if congregation_system.start_congregation(leader_id):
                        print(f"Congregación iniciada con líder {leader_id}")
                    else:
                        print(f"Error al iniciar congregación con líder {leader_id}")
                        
                elif command_part == 'STOP':
                    congregation_system.stop_congregation()
                    print("Congregación detenida")
                    
                elif command_part == 'STATUS':
                    status = congregation_system.get_congregation_status()
                    print(f"Estado de congregación: {status}")
                    
                else:
                    print("Comando de congregación no válido")
                continue
            
            # Procesamiento normal de instrucciones
            try:
                robotId, instruction = map(str.strip, instructionRaw.split('.', 1))
            except ValueError:
                print("Formato inválido. Use 'robotId.instrucción' o 'CONGREGATION.comando'")
                continue
            
            if robotId == 'BROADCAST':
                base.sendInstructionBroadcast([instruction])
            elif robotId in base.robots:
                robotIP = base.robots[robotId].IP
                base.sendInstruction(robotIP, [instruction], True)
            else:
                print(f"Robot ID '{robotId}' no encontrado.")
        
        base.threadInputAlive = False
    
    # Reemplazar el método original
    base.inputInstruction = enhanced_inputInstruction
    
    return congregation_system