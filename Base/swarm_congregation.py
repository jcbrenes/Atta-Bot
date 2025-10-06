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
        
        print("Sistema de congregación inicializado")
    
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
        # Buscar qué robot tiene esta IP
        requesting_robot = None
        for robot in self.base.robots.values():
            if robot.IP == sender_ip:
                requesting_robot = robot
                break
        
        if not requesting_robot:
            print(f"IP no reconocida en solicitud de posición: {sender_ip}")
            return
        
        # Obtener posición actual del robot desde el sistema de visión
        current_pose = requesting_robot.getPose()
        
        if current_pose == (-1, -1, -1):
            print(f"No se pudo obtener posición para robot {requesting_robot.id}")
            # Intentar usar la posición anterior si está disponible
            if requesting_robot.previousPose:
                current_pose = requesting_robot.previousPose
            else:
                return
        
        x, y, angle = current_pose
        
        # Enviar respuesta con posición directamente a la IP del robot
        response = f"POSITION_RESPONSE|{x}|{y}|{angle}"
        self.base.sendInstruction(sender_ip, [response], False)
        
        print(f"Posición enviada a robot {requesting_robot.id} ({requesting_robot.name}): {x}, {y}, {angle}")
    
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
            # Buscar robot por IP
            robot = self.find_robot_by_ip(sender_ip)
            robot_name = robot.name if robot else "unknown"
            print(f"Robot {robot_name} confirmó recepción de coordenadas")
            return True
            
        elif command == "ALL_MOVEMENTS_COMPLETE":
            # Buscar robot por IP
            robot = self.find_robot_by_ip(sender_ip)
            robot_name = robot.name if robot else "unknown"
            print(f"Robot {robot_name} completó movimientos de congregación")
            return True
            
        elif command == "LEADER_POSITION":
            # Los robots intercambian información de líder - solo logging
            leader_id = parts[1] if len(parts) > 1 else "unknown"
            robot = self.find_robot_by_ip(sender_ip)
            robot_name = robot.name if robot else "unknown"
            print(f"Robot {robot_name} anunció posición como líder {leader_id}")
            return True
        
        return False
    
    def find_robot_by_ip(self, ip):
        """
        Busca un robot por su dirección IP.
        
        Args:
            ip (str): Dirección IP del robot
            
        Returns:
            Robot: Objeto robot o None si no se encuentra
        """
        for robot in self.base.robots.values():
            if robot.IP == ip:
                return robot
        return None
    
    def stop_congregation(self):
        """
        Detiene el proceso de congregación activo.
        """
        self.congregation_active = False
        self.current_leader_id = None
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
            'total_robots': len(self.base.robots),
            'robots': {robot.id: {'name': robot.name, 'ip': robot.IP} for robot in self.base.robots.values()}
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
    
    # Guardar referencia al método original readUdpConnection
    original_readUdpConnection = base.readUdpConnection.__func__
    
    def enhanced_readUdpConnection(self):
        """
        Versión mejorada de readUdpConnection que incluye manejo de congregación.
        """
        self.createConcoleLog()
        self.sock.settimeout(None)
        
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                ip = addr[0]
                
                if ip != self.baseIP:
                    message = data.decode()
                    timeLog = round(time.time() - self.startTime, 1)
                    
                    # Intentar procesar mensaje de congregación primero
                    if congregation_system.handle_congregation_message(message, ip):
                        # Mensaje de congregación procesado, no hacer logging normal
                        continue
                    
                    # Procesamiento normal para otros mensajes
                    robot_info = next(
                        ((robot.name, robot.id) for robot in self.robots.values() if robot.IP == ip), 
                        (ip, "unknown")
                    )
                    name, robot_id = robot_info
                    self.addConcoleLog(timeLog, robot_id, name, message)
                    
                    if message.split('|')[0] == 'CHECK_OBSTACLE':
                        continue
                        
                    print(f"Mensaje de {name}: {message}")
                    
            except Exception as e:
                print(f"Error en comunicación UDP: {e}")
                break
    
    # Reemplazar el método con la versión mejorada usando bound method
    base.readUdpConnection = enhanced_readUdpConnection.__get__(base, base.__class__)
    
    # Guardar referencia al método original inputInstruction
    original_inputInstruction = base.inputInstruction.__func__
    
    def enhanced_inputInstruction(self):
        """
        Versión mejorada de inputInstruction que incluye comandos de congregación.
        """
        print("\n=== COMANDOS DISPONIBLES ===")
        print("Comandos normales:")
        print("  - robotId.instrucción (ej: 1.MOVE|100)")
        print("  - BROADCAST.instrucción")
        print("\nComandos de congregación:")
        print("  - CONGREGATION.START|leader_id (ej: CONGREGATION.START|4)")
        print("  - CONGREGATION.STOP")
        print("  - CONGREGATION.STATUS")
        print("\nOtros:")
        print("  - BREAK (para salir)")
        print("=============================\n")
        
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
                        print(f"✓ Congregación iniciada con líder {leader_id}")
                    else:
                        print(f"✗ Error al iniciar congregación con líder {leader_id}")
                        
                elif command_part == 'STOP':
                    congregation_system.stop_congregation()
                    print("✓ Congregación detenida")
                    
                elif command_part == 'STATUS':
                    status = congregation_system.get_congregation_status()
                    print(f"\n--- ESTADO DE CONGREGACIÓN ---")
                    print(f"Activa: {status['active']}")
                    print(f"Líder: {status['leader_id']}")
                    print(f"Total robots: {status['total_robots']}")
                    if status['robots']:
                        print("Robots disponibles:")
                        for robot_id, robot_info in status['robots'].items():
                            print(f"  {robot_id}: {robot_info['name']} ({robot_info['ip']})")
                    print("------------------------------\n")
                    
                else:
                    print("✗ Comando de congregación no válido")
                    print("Comandos válidos: START|id, STOP, STATUS")
                continue
            
            # Procesamiento normal de instrucciones
            try:
                robotId, instruction = map(str.strip, instructionRaw.split('.', 1))
            except ValueError:
                print("✗ Formato inválido. Use 'robotId.instrucción' o 'CONGREGATION.comando'")
                continue
            
            if robotId == 'BROADCAST':
                self.sendInstructionBroadcast([instruction])
            elif robotId in self.robots:
                robotIP = self.robots[robotId].IP
                self.sendInstruction(robotIP, [instruction], True)
            else:
                print(f"✗ Robot ID '{robotId}' no encontrado.")
                available_robots = ', '.join(self.robots.keys())
                print(f"Robots disponibles: {available_robots}")
        
        self.threadInputAlive = False
    
    # Reemplazar el método con la versión mejorada usando bound method
    base.inputInstruction = enhanced_inputInstruction.__get__(base, base.__class__)
    
    return congregation_system