import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'  # forzar xcb — cv2 no tiene plugin wayland
import cv2, json, math, time, socket, threading, csv, multiprocessing, threading, platform
import numpy as np
import readline
from datetime import datetime

# GUI opcional — se usa cuando se llama vía AttaBot_GUI.launch()
_gui_instance = None


def runOnThread(func):
    """
    Ejecuta una función en un hilo separado, permitiendo que el código principal
    continúe sin bloquearse.

    La función decorada se ejecutará en un hilo en segundo plano,
    usando el modo daemon para que el hilo termine automáticamente al finalizar
    el programa principal.

    Parámetros:
    func (callable): La función que se desea ejecutar en un hilo.

    Retorna:
    function: Un decorador que inicia un hilo para ejecutar la función `func`
    con los argumentos proporcionados y devuelve el hilo en ejecución.
    """
    def wrapper(*args, **kwargs):
        def executeFunc():
            func(*args, **kwargs)

        thread = threading.Thread(target=executeFunc, daemon=True)
        thread.start()

        return thread

    return wrapper


# Colores BGR por ID de robot para el mapa de cobertura
_ROBOT_COLORS_BGR = [
    (220,  70,  30),  # 0: azul
    ( 30, 180,  30),  # 1: verde
    ( 30,  30, 210),  # 2: rojo
    (190,  40, 190),  # 3: magenta
    ( 20, 190, 190),  # 4: amarillo
    ( 20, 130, 220),  # 5: naranja
]


def videoWriter(frameResolution, numRobots, pathVideo, processInterval, queue, debugResolution):
    """
    Graba un video en el disco utilizando los cuadros de video que llegan a través de una cola,
    mostrando además el video en una ventana.

    Este proceso lee pares de fotogramas de `queue`, une los fotogramas en uno solo y
    los guarda en un archivo AVI con un nombre específico que incluye la cantidad de robots y la fecha y hora actuales.
    También muestra el video en pantalla.

    Parámetros:
    frameResolution (tuple): Resolución original de los fotogramas de entrada (ancho, alto).
    numRobots (int): Número de robots, usado para el nombre del archivo de video.
    pathVideo (str): Directorio donde se guardará el video generado.
    processInterval (float): Intervalo de procesamiento en segundos para calcular los FPS.
    queue (Queue): Cola que contiene los fotogramas a grabar, en formato `(frame, resultsFrame)`.
    debugResolution (tuple): Resolución para mostrar el video de depuración (ancho, alto).

    Retorna:
    None
    """
    currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
    videoName = f'Video_{currentTime}_Robots_{numRobots}.avi'
    pathVideo = os.path.join(pathVideo, videoName)
    resolution = (frameResolution[1], frameResolution[0] * 2)
    fps = 1 / processInterval - 1
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video = cv2.VideoWriter(pathVideo, fourcc, fps, resolution)

    while True:
        frames = queue.get()

        if frames is None:
            break

        frame, resultsFrame = frames

        # Solo grabación — el display lo maneja la GUI o el proceso principal
        results = cv2.vconcat([frame, resultsFrame])
        video.write(results)

    video.release()


# =============================================================================
# CLASE COLOR ELIMINADA
# El sistema de detección por círculos de color HSV fue reemplazado por
# detección de ArUco markers. La identificación del robot se realiza
# directamente por el ID del marker, eliminando la necesidad de segmentación
# por color y el cálculo de centroides.
# =============================================================================


class Robot(object):
    """
    Clase que representa un robot en un sistema de enjambre.

    La detección de pose se realiza mediante ArUco markers.
    El ID del marker ArUco corresponde directamente al ID del robot.

    Atributos:
        id (str): Identificador único del robot (igual al ID del marker ArUco).
        name (str): Nombre del robot.
        IP (str): Dirección IP del robot para la comunicación.
        previousPose (tuple): Última posición conocida del robot en formato (x, y, angle).
    """

    def __init__(self, id, configRobot):
        self.id = id
        self.name = ''
        self.IP = ''
        self.previousPose = (-1, -1, -1)
        self.initRobot(configRobot)


    def getPose(self):
        """
        Obtiene la pose actual del robot desde el diccionario de detecciones ArUco.

        El ID del marker ArUco coincide con el ID del robot, por lo que la
        identificación es directa sin necesidad de comparar colores ni distancias.

        Returns:
            tuple: (x_mm, y_mm, angle_deg) si el robot es visible, (-1, -1, -1) si no.
        """
        if self.id in base.currentArucoDetections:
            return base.currentArucoDetections[self.id]
        return (-1, -1, -1)


    def initRobot(self, configRobot):
        """
        Inicializa el robot configurando su nombre y establece su pose inicial.

        Args:
            configRobot (dict): Diccionario de configuración de los robots.
        """
        self.name = configRobot[self.id]['name']
        self.angleOffset = float(configRobot[self.id].get('angle_offset', 0.0))
        wd = configRobot[self.id].get('wheel_distance')
        self.wheelDistance = float(wd) if wd is not None else None
        self.previousPose = self.getPose()


    def getDisplacement(self):
        """
        Calcula el desplazamiento lineal y angular del robot en función de su
        pose actual y anterior.

        Este método obtiene la pose actual del robot y si es válida, calcula
        la diferencia entre la pose actual y la anterior. Devuelve el desplazamiento
        lineal y angular si estas diferencias superan un umbral definido.

        Returns:
            tuple or None: Un tuple que contiene el desplazamiento lineal y
                            angular en milímetros y grados respectivamente,
                            o None si la pose actual no es válida o si los
                            desplazamientos son menores al umbral.

        Note:
            Se considera que un desplazamiento es significativo si la diferencia
            angular es mayor o igual a 4 grados o si la diferencia lineal es
            mayor o igual a 4 milímetros.
        """
        currentPose = self.getPose()
        if currentPose == (-1, -1, -1):
            return None

        x1, y1, a1 = self.previousPose
        x2, y2, a2 = currentPose

        linearDisp = self.linearDisplacement(x1, y1, a1, x2, y2)
        angularDisp = self.angularDisplacement(a1, a2)

        if abs(angularDisp) >= 4 or abs(linearDisp) >= 4:
            self.previousPose = currentPose
            return linearDisp, angularDisp

        return None


    def angularDisplacement(self, a1, a2):
        """
        Calcula el desplazamiento angular entre dos ángulos.

        Asegura que el resultado se mantenga en el rango de -180 a 180 grados.

        Args:
            a1 (float): El ángulo inicial en grados.
            a2 (float): El ángulo final en grados.

        Returns:
            float: El desplazamiento angular en grados, re1dondeado a un decimal.
        """
        angularDisplacement = a2 - a1
        if angularDisplacement > 180:
            angularDisplacement -= 360
        elif angularDisplacement < -180:
            angularDisplacement += 360

        return round(angularDisplacement, 1)


    def linearDisplacement(self, x1, y1, a1, x2, y2):
        """
        Calcula el desplazamiento lineal entre dos posiciones dadas,
        teniendo en cuenta la dirección del ángulo de la primera posición.

        Args:
            x1 (float): Coordenada x de la primera posición.
            y1 (float): Coordenada y de la primera posición.
            a1 (float): Ángulo en grados de la primera posición.
            x2 (float): Coordenada x de la segunda posición.
            y2 (float): Coordenada y de la segunda posición.

        Returns:
            float: El desplazamiento lineal entre las dos posiciones, redondeado a
                un decimal. Positivo = misma dirección que el ángulo.
        """
        a1Rad = math.radians(a1)
        dx = x2 - x1
        dy = y2 - y1

        dotProduct = dx * math.cos(a1Rad) + dy * math.sin(a1Rad)
        displacement = round(math.dist((x1, y1), (x2, y2)), 1)

        return displacement if dotProduct >= 0 else -displacement


    def setupIP(self, ip):
        """
        Configura la dirección IP del robot y envía una instrucción de configuración.

        Args:
            ip (str): La dirección IP que se asignará al robot.
        """
        self.IP = ip
        instructions = [f'CONFIG|{self.id}']
        if self.wheelDistance is not None:
            instructions.append(f'NAV_CONFIG|WHEEL_DIST|{self.wheelDistance}')
        base.sendInstruction(ip, instructions, False)



class Base(object):
    """
    Clase Base para la gestión de un sistema de robots de enjambre con detección ArUco.

    El sistema de visión utiliza markers ArUco (DICT_4X4_50) para identificar y
    localizar los robots. Cada robot lleva un marker cuyo ID corresponde al ID del robot.
    La pose (x, y, ángulo) se calcula con solvePnP a partir de las esquinas detectadas.

    Atributos principales:
        arucoDetector: Detector de markers ArUco inicializado con DICT_4X4_50.
        markerSizeMm (float): Tamaño físico del marker en mm (configurado en JSON).
        currentArucoDetections (dict): {robot_id (str): (x_mm, y_mm, angle_deg)}
            Actualizado en cada frame procesado. Accedido por Robot.getPose().
        robots (dict): Diccionario {id: Robot} de robots activos.
        [resto de atributos igual que antes]
    """

    def __init__(self):
        self.robots = {}
        self.robotsConfig = {}
        self.numRobots = int
        self.debug = False
        self.camera = None
        self.cameraIndex = None
        self.cameraBackend = None
        self.debugResolution = tuple
        self.mmPixel = float
        self.processInterval = float
        self.startTime = float
        self.cameraMatriz = []
        self.distance = []
        self.sock = None
        self.baseIP = ''
        self.broadcastIP = ''
        self.port = int
        self.threadInputAlive = True
        self.pathVideo = ''
        self.pathPositionLogs = ''
        self.pathConsolelog = ''
        self.cameraResolution = []
        self.newCameraMatriz = None
        self.roi = None
        self.frameQueue = None
        self.videoProcess = multiprocessing.Process()
        self.congregationActive = False
        self.leaderID = None
        self.robotPositions = {}
        # --- ArUco ---
        self.arucoDetector = None
        self.markerSizeMm = 80.0          # valor por defecto, sobreescrito desde JSON
        self.currentArucoDetections = {}    # raw: {robot_id: (x_mm, y_mm, angle_deg)}
        self._smoothedArucoDetections = {} # EMA-suavizado, solo para enviar posiciones al robot
        self._arucoEma = {}               # estado interno del EMA
        self.arucoEmaAlpha = 0.4          # peso del frame nuevo (0=sin cambio, 1=sin suavizado)
        self.bigCircleRadius = 10         # radio visual en el resultsFrame (px)
        self.cellSizeMm = 50.0            # tamaño de celda del mapa de cobertura en mm
        self.coverageGrid = None          # grilla de cobertura: -1=libre, else robot_id
        self.cellPx = 1                   # tamaño de celda en píxeles
        self.gui = None                   # referencia a AttaBotGUI (None = modo terminal)


    def log(self, msg: str):
        """Muestra un mensaje en el log de la GUI o en la terminal si no hay GUI."""
        if self.gui is not None:
            self.gui.logSignal.emit(msg)
        else:
            print(msg)


    # =========================================================================
    # DETECCIÓN ARUCO
    # =========================================================================

    def detectArucoMarkers(self, frame):
        """
        Detecta ArUco markers en el frame BGR y retorna poses en mm y grados.

        Reemplaza completamente el sistema de detección por círculos de color.
        Usa solvePnP con SOLVEPNP_IPPE_SQUARE para obtener posición 3D y orientación
        de cada marker visible. Las coordenadas se expresan en el plano de la cámara:
            - X positivo: derecha
            - Y positivo: abajo
            - El ángulo es el heading del robot en el plano XY (0° = derecha, CW positivo)

        Parámetros:
        - frame (ndarray): Frame BGR de la cámara ya corregido por distorsión.

        Retorna:
        - dict: {marker_id (str): (x_mm, y_mm, angle_deg)}
                Los IDs son strings para compatibilidad con el resto del sistema.
                Retorna {} si no se detecta ningún marker.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detección a resolución completa: a 2.5m de altura el marker de 80mm ocupa
        # solo ~47px — a media resolución baja a ~24px (3.9px/celda), límite de fallo.
        corners, ids, _ = self.arucoDetector.detectMarkers(gray)

        detectedPoses = {}

        if ids is None:
            return detectedPoses

        # Puntos 3D del marker en coordenadas locales (metros)
        # Orden: top-left, top-right, bottom-right, bottom-left
        halfSize = (self.markerSizeMm / 1000.0) / 2.0
        objectPoints = np.array([
            [-halfSize,  halfSize, 0.0],
            [ halfSize,  halfSize, 0.0],
            [ halfSize, -halfSize, 0.0],
            [-halfSize, -halfSize, 0.0]
        ], dtype=np.float32)

        # Paso 1: resolver pose de todos los markers y buscar el de referencia
        rawPositions = {}  # {str(id): (raw_x_mm, raw_y_mm, angle_deg)}
        for i, marker_id in enumerate(ids.flatten()):
            imagePoints = corners[i][0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                objectPoints, imagePoints,
                self.cameraMatriz, self.distance,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not success:
                continue
            raw_x = float(tvec[0][0]) * 1000.0
            raw_y = float(tvec[1][0]) * 1000.0
            rotMatrix, _ = cv2.Rodrigues(rvec)
            angle_rad = np.arctan2(rotMatrix[1][0], rotMatrix[0][0])
            angle_deg = round(float(np.degrees(angle_rad) % 360), 1)
            rawPositions[str(marker_id)] = (raw_x, raw_y, angle_deg)

        # Paso 2: si hay marker de referencia visible, anclar origen a él
        if self.referenceMarkerId and self.referenceMarkerId in rawPositions:
            ref_x, ref_y, _ = rawPositions[self.referenceMarkerId]
            self.originXmm = round(ref_x, 1)
            self.originYmm = round(ref_y, 1)

        # Paso 3: calcular poses relativas al origen
        for marker_id_str, (raw_x, raw_y, angle_deg) in rawPositions.items():
            if marker_id_str == self.referenceMarkerId:
                detectedPoses[marker_id_str] = (0.0, 0.0, angle_deg)
                continue
            x_mm = round(raw_x - self.originXmm, 1)
            y_mm = round(raw_y - self.originYmm, 1)
            # Aplicar offset de ángulo por robot (compensa marker montado rotado)
            offset = 0.0
            if marker_id_str in self.robots:
                offset = self.robots[marker_id_str].angleOffset
            corrected_angle = round((angle_deg + offset) % 360, 1)
            detectedPoses[marker_id_str] = (x_mm, y_mm, corrected_angle)

        return detectedPoses


    def drawArucoDebug(self, frame):
        """
        Dibuja los markers detectados sobre el frame para depuración visual.

        Muestra los ejes de coordenadas de cada marker y su ID.
        Solo se llama cuando self.debug está activado.

        Parámetros:
        - frame (ndarray): Frame BGR donde dibujar las anotaciones.

        Retorna:
        - ndarray: Frame anotado.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.arucoDetector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            halfSize = (self.markerSizeMm / 1000.0) / 2.0
            objectPoints = np.array([
                [-halfSize,  halfSize, 0.0],
                [ halfSize,  halfSize, 0.0],
                [ halfSize, -halfSize, 0.0],
                [-halfSize, -halfSize, 0.0]
            ], dtype=np.float32)

            for i, marker_id in enumerate(ids.flatten()):
                imagePoints = corners[i][0].astype(np.float32)
                success, rvec, tvec = cv2.solvePnP(
                    objectPoints, imagePoints,
                    self.cameraMatriz, self.distance,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if success:
                    cv2.drawFrameAxes(frame, self.cameraMatriz, self.distance,
                                      rvec, tvec, self.markerSizeMm / 1000.0 * 0.5)

                    # Texto con pose encima del marker
                    cx = int(np.mean(corners[i][0][:, 0]))
                    cy = int(np.mean(corners[i][0][:, 1]))
                    if str(marker_id) in self.currentArucoDetections:
                        x_mm, y_mm, ang = self.currentArucoDetections[str(marker_id)]
                        label = f'ID:{marker_id} ({x_mm},{y_mm}) {ang}deg'
                        cv2.putText(frame, label, (cx - 60, cy - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 100), 1, cv2.LINE_AA)

        return frame


    # =========================================================================
    # BÚSQUEDA Y SETUP DE ROBOTS
    # =========================================================================

    def searchRobotsAruco(self, robots):
        """
        Detecta qué robots están visibles en el frame actual por su marker ArUco.

        Reemplaza searchRobotsColor. Con ArUco el ID del marker es directamente el
        ID del robot, por lo que no se necesita correlación de colores ni distancias.

        Parámetros:
        - robots (dict): Configuración de robots desde el JSON.

        Retorna:
        - foundRobots (set): IDs (str) de robots detectados visualmente.
        - foundColors (set): Set vacío — mantenido por compatibilidad con processFoundRobots.
        """
        foundRobots = set()

        for robotId in robots.keys():
            # El marker de referencia (origen del escenario) no es un robot
            if robotId == self.referenceMarkerId:
                continue
            if robotId in self.currentArucoDetections:
                foundRobots.add(robotId)

        return foundRobots, set()


    def processFoundRobots(self, foundRobots):
        """
        Crea instancias de Robot para cada robot encontrado y los ordena por ID.

        Versión simplificada para ArUco: ya no necesita eliminar colores no usados
        porque no existe el diccionario frameColors.

        Parámetros:
        - foundRobots (set): IDs de robots detectados visualmente.
        """
        self.robots = {robot: Robot(robot, self.robotsConfig) for robot in sorted(foundRobots, key=int)}
        if self.gui is not None:
            self.gui.refreshRobots()


    def setupRobots(self, robotIP, robotsIPs, configuredRobots):
        """
        Asocia una dirección IP a un robot en función de la detección del sistema de visión.

        Si no se proporciona una dirección IP se selecciona una de robotsIPs, después se
        gira el robot para identificar cuál se mueve y se le asigna la dirección IP
        seleccionada.

        Parameters:
        - robotIP (str o None): Dirección IP actual del robot.
        - robotsIPs (list): Lista de direcciones IP disponibles para asignar a los robots.
        - configuredRobots (set): Conjunto de IDs de robots ya configurados.

        Returns:
        - tuple: (robotIP, isValidFrame)
        """
        if robotIP is None:
            # Snapshot del ángulo de cada robot ANTES del giro de identificación.
            # Comparar contra el snapshot (y no frame-a-frame) evita que el giro
            # se vea "en cachitos" <45° cuando hay frames atrasados en el buffer.
            self._setupAngleSnapshot = {}
            for robot in self.robots.values():
                pose = robot.getPose()
                if pose != (-1, -1, -1):
                    self._setupAngleSnapshot[robot.id] = pose[2]
            robotIP = self.setupMoveRobot(robotsIPs)
            isValidFrame = False
        else:
            prevLen = len(robotsIPs)
            self.setupRobotIP(robotsIPs, configuredRobots)
            robotIP = None if len(robotsIPs) < prevLen else robotIP
            isValidFrame = True

        if len(robotsIPs) == 0:
            self.printRobots()
            time.sleep(0.5)

        return robotIP, isValidFrame


    def printRobots(self):
        """
        Imprime la lista de robots encontrados y envía una instrucción para que
        cada robot gire 90 grados en sentido antihorario.
        """
        print('Robots encontrados: ')
        for robot in self.robots.values():
            self.sendInstruction(robot.IP, ['TURN|-90'], False)
            print(f"\t{robot.id}. {robot.name}, con IP: {robot.IP}")


    def setupRobotIP(self, robotsIPs, configuredRobots):
        """
        Asocia una dirección IP a un robot en función de su desplazamiento angular.

        Con ArUco, el desplazamiento se detecta directamente desde el cambio de ángulo
        del marker, sin necesidad de comparar círculos de color.

        Parameters:
        - robotsIPs (list): Lista de direcciones IP disponibles para los robots.
        - configuredRobots (set): Conjunto de IDs de robots ya configurados.

        Returns:
        - None
        """
        for robot in self.robots.values():
            if robot.id in configuredRobots:
                continue

            pose = robot.getPose()
            refAngle = getattr(self, '_setupAngleSnapshot', {}).get(robot.id)

            now = time.time()
            if now - getattr(self, '_setupDbgTime', 0) >= 1.0:
                self._setupDbgTime = now
                visible = pose != (-1, -1, -1)
                print(f"[Setup-dbg] Robot {robot.id}: visible={visible} "
                      f"ref={refAngle} actual={pose[2] if visible else '—'} "
                      f"disp={robot.angularDisplacement(refAngle, pose[2]) if (visible and refAngle is not None) else '—'}")

            if pose == (-1, -1, -1) or refAngle is None:
                continue

            angularDisp = robot.angularDisplacement(refAngle, pose[2])
            if abs(angularDisp) >= 45:
                robotIP = robotsIPs.pop()
                robot.setupIP(robotIP)
                configuredRobots.add(robot.id)
                break

        return None


    def setupMoveRobot(self, robotsIPs):
        """
        Envía instrucciones para girar al último robot de la lista robotsIPs.

        Parameters:
        - robotsIPs (list): Lista de direcciones IP disponibles para los robots.

        Returns:
        - str: La dirección IP asignada al robot.
        """
        robotIP = robotsIPs[-1]
        instructions = ['TURN|90', 'MESSAGE_BASE|1']
        self.sendInstruction(robotIP, instructions, False)

        self.sock.settimeout(0.5)
        deadline = time.time() + 8.0
        while time.time() < deadline:
            try:
                data, addr = self.sock.recvfrom(1024)
                if addr[0] != self.baseIP and robotIP == addr[0]:
                    message = data.decode()
                    print(f"Respuesta de {addr[0]}: {message}")
                    if message == 'READY':
                        print(f"[Setup] Robot {robotIP} listo")
                        break
            except socket.timeout:
                pass

        return robotIP


    # =========================================================================
    # CONFIGURACIÓN
    # =========================================================================

    def readConfigFile(self, filePath):
        """
        Lee un archivo de configuración JSON y aplica las configuraciones del sistema.

        Cambios respecto al sistema de círculos:
        - Se elimina la sección 'colors' del JSON.
        - Se elimina 'circles_radius' y 'distance_between_centers'.
        - Se agrega 'marker_size_mm' en vision_system.
        - Se elimina el campo 'colors' de cada robot en la sección 'robots'.

        Parámetros:
        - filePath (str): La ruta del archivo de configuración JSON.
        """
        with open(filePath, 'r') as file:
            configuration = json.load(file)

        self.configVisionSystem(configuration['vision_system'])
        self.configUdp(configuration['udp_communication'])
        self.generalConfig(configuration['general'])

        self.robotsConfig = configuration['robots']


    def generalConfig(self, configuration):
        """
        Configura las opciones generales del sistema a partir de un diccionario de configuración.

        Parámetros:
        - configuration (dict): Configuración general del sistema.
        """
        self.debug = configuration['debug_enable']

        self.pathVideo = configuration['path_save_videos']
        self.pathPositionLogs = configuration['path_save_position_logs']
        self.pathConsolelog = configuration['path_save_console_logs']

        for path in [self.pathVideo, self.pathPositionLogs, self.pathConsolelog]:
            if not os.path.exists(path):
                print(f"La ruta {path} no existe.")
                try:
                    os.makedirs(path)
                    print(f"Se ha creado la ruta {path}.")
                except Exception as e:
                    raise Exception(f"Error al crear la ruta: {e}")


    def configUdp(self, configuration):
        """
        Configura la comunicación UDP para el sistema.

        Parámetros:
        - configuration (dict): Configuración UDP.
        """
        self.baseIP = configuration['base_ip']
        self.broadcastIP = configuration['broadcast_ip']
        self.port = configuration['port']
        self.networkInterface = configuration.get('network_interface', None)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        if platform.system() == 'Linux':
            try:
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                print('✓ SO_REUSEPORT habilitado (Linux)')
            except AttributeError:
                print('⚠ SO_REUSEPORT no disponible en esta versión de Python')

            if self.networkInterface:
                try:
                    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                                         self.networkInterface.encode())
                    print(f'✓ Socket vinculado a interfaz {self.networkInterface}')
                except (AttributeError, OSError) as e:
                    print(f'⚠ No se pudo vincular a interfaz {self.networkInterface}: {e}')

        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**20)

        # Bind en 0.0.0.0 para evitar [Errno 99] cuando base_ip no está asignada
        # a la interfaz en el momento del bind. SO_BINDTODEVICE (Linux) ya restringe
        # el tráfico a la interfaz correcta (enp3s0), por lo que el bind amplio es seguro.
        bindAddress = '0.0.0.0' if platform.system() == 'Linux' else self.baseIP
        try:
            self.sock.bind((bindAddress, self.port))
            print(f'✓ Socket UDP bind exitoso en {bindAddress}:{self.port}')
            if bindAddress == '0.0.0.0':
                print(f'  (Tráfico restringido a interfaz {self.networkInterface or "todas"})')
        except OSError as e:
            print(f'✗ Error al hacer bind en {bindAddress}:{self.port}')
            print(f'  Motivo: {e}')
            raise


    def configVisionSystem(self, configuration):
        """
        Configura el sistema de visión con soporte ArUco.

        Cambios respecto al sistema de círculos:
        - Inicializa el detector ArUco con DICT_4X4_50.
        - Lee marker_size_mm desde la configuración.
        - Elimina la configuración de circles_radius y distance_between_centers.

        Parámetros:
        - configuration (dict): Configuración del sistema de visión.
            Claves requeridas:
                'path_cameraMatrix', 'path_distance', 'mmPixel',
                'frame_processing_interval', 'marker_size_mm'
        """
        self.setCamera(configuration)

        self.cameraMatriz = np.loadtxt(configuration['path_cameraMatrix'], dtype=float)
        self.distance = np.loadtxt(configuration['path_distance'], dtype=float)
        h, w = self.cameraResolution

        # La calibración se hizo a 1920x1080. Escalar la matriz si la resolución cambió.
        calib_w, calib_h = 1920, 1080
        sx, sy = w / calib_w, h / calib_h
        if sx != 1.0 or sy != 1.0:
            self.cameraMatriz = self.cameraMatriz.copy()
            self.cameraMatriz[0, 0] *= sx  # fx
            self.cameraMatriz[1, 1] *= sy  # fy
            self.cameraMatriz[0, 2] *= sx  # cx
            self.cameraMatriz[1, 2] *= sy  # cy
            print(f'  Matriz de cámara escalada {calib_w}x{calib_h} → {w}x{h}')

        self.newCameraMatriz, self.roi = cv2.getOptimalNewCameraMatrix(
            self.cameraMatriz, self.distance, (w, h), 1, (w, h)
        )
        # Pre-computar mapas de corrección — remap es 3-5x más rápido que undistort
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.cameraMatriz, self.distance, None, self.newCameraMatriz, (w, h), cv2.CV_16SC2
        )

        numerator, denominator = map(int, configuration['mmPixel'].split('/'))
        self.mmPixel = numerator / denominator
        self.processInterval = configuration['frame_processing_interval']

        # Tamaño físico del marker en mm (recomendado: 80mm+ a 2.5m de distancia)
        self.markerSizeMm = float(configuration['marker_size_mm'])
        self.originXmm = float(configuration.get('origin_x_mm', 0))
        self.originYmm = float(configuration.get('origin_y_mm', 0))
        # ID del marker fijo de referencia que ancla el origen del escenario.
        # Si está presente en el frame, el sistema lo usa como (0,0) automáticamente.
        ref = configuration.get('reference_marker_id', '')
        self.referenceMarkerId = str(ref) if ref != '' else None

        # Radio visual para el resultsFrame (proporcional al tamaño del marker en px)
        self.bigCircleRadius = max(6, int((self.markerSizeMm / self.mmPixel) * 0.5))

        # Inicializar detector ArUco
        # DICT_4X4_50: markers 4x4 bits, 50 IDs disponibles — robusto y compacto
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()

        # A 2.5m de altura los markers de 80mm miden ~47px a resolución completa.
        # WinSize: ventana adaptativa relativa al tamaño del marker — mín 3, máx ~marker/2
        arucoParams.adaptiveThreshWinSizeMin = 3
        arucoParams.adaptiveThreshWinSizeMax = 23    # ~marker/2; antes era 53 (para media-res)
        arucoParams.adaptiveThreshWinSizeStep = 4
        arucoParams.adaptiveThreshConstant = 7       # default; con full-res hay más señal
        arucoParams.minMarkerPerimeterRate = 0.02    # 47px perím. / 1280px ancho ≈ 0.037 — margen
        arucoParams.maxMarkerPerimeterRate = 4.0
        arucoParams.polygonalApproxAccuracyRate = 0.05
        arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        arucoParams.errorCorrectionRate = 0.9
        arucoParams.perspectiveRemovePixelPerCell = 8

        self.arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
        print(f'✓ Detector ArUco inicializado — DICT_4X4_50, marker: {self.markerSizeMm}mm')


    def setCamera(self, configuration):
        """
        Inicializa la captura de video desde la cámara.

        Parámetros:
        - configuration (dict): Configuración de la cámara.
        """
        system = platform.system()
        if system == 'Linux':
            backend = cv2.CAP_V4L2
        elif system == 'Windows':
            backend = cv2.CAP_DSHOW
        elif system == 'Darwin':
            backend = cv2.CAP_AVFOUNDATION
        else:
            backend = cv2.CAP_ANY

        camera_index = configuration.get('camera_index', None)

        if camera_index is None:
            print(f'Detectando cámara en {system}...')
            for i in range(5):
                test_cam = cv2.VideoCapture(i, backend)
                if test_cam.isOpened():
                    camera_index = i
                    test_cam.release()
                    print(f'✓ Cámara encontrada en índice {i}')
                    break

            if camera_index is None:
                print('Error: No se detectó ninguna cámara.')
                exit()

        self.cameraIndex = camera_index
        self.cameraBackend = backend
        self.camera = cv2.VideoCapture(camera_index, backend)
        print(f'Backend de cámara: {backend} (índice {camera_index})')

        self.debugResolution = tuple(map(int, configuration['debug_resolution'].split('x')))
        width, height = map(int, configuration['camera_resolution'].split('x'))
        self.cameraResolution = (height, width)

        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # evita acumulación de frames viejos
        # MJPEG permite 1080p @ 30 FPS por USB; sin esto V4L2 usa YUYV (~5 FPS)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        # Fijar exposición para evitar parpadeo ("tweaking") bajo luz de laboratorio
        self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 1 = manual en V4L2
        self.camera.set(cv2.CAP_PROP_EXPOSURE, 200)

        actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
        actual_w   = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f'✓ Cámara: {actual_w}x{actual_h} @ {actual_fps:.0f} FPS')

        # Drena frames iniciales corruptos (MJPEG tarda ~30 frames en estabilizarse)
        print('  Calentando cámara...', end='', flush=True)
        for _ in range(30):
            self.camera.read()
        print(' listo')

        if not self.camera.isOpened():
            print(f'Error: No se pudo abrir la cámara {camera_index}.')
            for i in range(5):
                test = cv2.VideoCapture(i, backend)
                if test.isOpened():
                    print(f'  - /dev/video{i} (índice {i})')
                    test.release()
            exit()


    # =========================================================================
    # PROCESAMIENTO DE FRAMES
    # =========================================================================

    def _applyArucoEma(self, raw):
        alpha = self.arucoEmaAlpha
        smoothed = {}
        for rid, (x, y, angle) in raw.items():
            if rid not in self._arucoEma:
                self._arucoEma[rid] = (x, y, angle)
            ex, ey, ea = self._arucoEma[rid]
            nx = alpha * x + (1 - alpha) * ex
            ny = alpha * y + (1 - alpha) * ey
            # ángulo: EMA sobre diferencia normalizada para evitar salto 0/360
            diff = ((angle - ea) + 180) % 360 - 180
            na = (ea + alpha * diff) % 360
            self._arucoEma[rid] = (nx, ny, na)
            smoothed[rid] = (round(nx, 1), round(ny, 1), round(na, 1))
        # limpiar EMA de markers que dejaron de verse
        for rid in list(self._arucoEma):
            if rid not in raw:
                del self._arucoEma[rid]
        return smoothed

    def cameraCorrection(self, frame):
        """
        Desdistorsiona y recorta la imagen de la cámara.

        Parámetros:
        - frame (ndarray): La imagen de entrada en formato BGR.

        Returns:
        - frame (ndarray): Imagen corregida en formato BGR.
        - frameGray (ndarray): Imagen en escala de grises (para uso interno).
        """
        x, y, w, h = self.roi
        frame = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)[y:y+h, x:x+w]
        frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        return frame, frameGray


    def processFrame(self, frame):
        """
        Procesa un frame de la cámara: corrige distorsión, detecta markers ArUco
        y actualiza currentArucoDetections.

        Reemplaza processFrameColors. Ya no se procesan colores HSV ni se lanzan
        hilos por color — la detección ArUco opera sobre el frame BGR completo.

        Parámetros:
        - frame (ndarray): Frame BGR crudo de la cámara.

        Retorna:
        - frame (ndarray): Frame corregido por distorsión en BGR.
        - frameGray (ndarray): Frame en escala de grises.
        """
        frame, frameGray = self.cameraCorrection(frame)

        # Detección ArUco — raw para desplazamiento/setup, suavizado para navegación
        raw = self.detectArucoMarkers(frame)
        self.currentArucoDetections = raw
        self._smoothedArucoDetections = self._applyArucoEma(raw)

        if self.debug:
            self.cameraDebug(frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.debug = 'off'

        return frame, frameGray


    def cameraDebug(self, frame):
        """
        Muestra una ventana de depuración con los markers ArUco anotados.

        Parámetros:
        - frame (ndarray): Frame BGR ya corregido.
        """
        if self.gui is not None:
            return  # GUI recibe frames via frameSignal; no se necesita ventana separada
        debugFrame = self.drawArucoDebug(frame.copy())
        resized = cv2.resize(debugFrame, self.debugResolution, interpolation=cv2.INTER_AREA)
        cv2.imshow('Debug ArUco', resized)


    # =========================================================================
    # LOOP PRINCIPAL
    # =========================================================================

    def cameraProcessing(self):
        """
        Loop principal de procesamiento de cámara.

        Fases:
        1. Detección visual: busca markers ArUco hasta encontrar numRobots robots.
        2. Setup de red: asocia IPs a robots por desplazamiento angular.
        3. Operación: envía poses a los robots en cada frame.

        El flujo es idéntico al sistema de círculos, solo cambia la fuente de
        detección (ArUco en lugar de HSV + contornos).
        """
        robotIP = None
        isValidFrame = True
        robotsIPs = []
        foundRobots = set()
        configuredRobots = set()
        lastProcessedTime = time.time()
        lastStatusTime = time.time()
        executed = False
        setupRetries = 0
        MAX_SETUP_RETRIES = 15

        failCount = 0

        while True:
            ret, frame = self.camera.read()

            if not ret or frame is None:
                failCount += 1
                if failCount % 30 == 1:
                    print(f"[CAM] Fallo de lectura (#{failCount}), reintentando...")
                if failCount >= 30:
                    print("[CAM] Reabriendo cámara...")
                    self.camera.release()
                    self.camera = cv2.VideoCapture(self.cameraIndex, self.cameraBackend)
                    failCount = 0
                continue

            failCount = 0

            if time.time() - lastProcessedTime < self.processInterval or not isValidFrame:
                isValidFrame = True
                if (self.debug == 'off' or not self.threadInputAlive or
                        not self.videoProcess.is_alive()) and executed:
                    self.cleanup()
                    break
                continue

            lastProcessedTime = time.time()
            frame, frameGray = self.processFrame(frame)

            if len(foundRobots) < self.numRobots:
                foundRobots, _ = self.searchRobotsAruco(self.robotsConfig)

                if time.time() - lastStatusTime >= 2.0:
                    lastStatusTime = time.time()
                    visible = list(self.currentArucoDetections.keys())
                    print(f"[Búsqueda] Robots detectados: {sorted(foundRobots)} / "
                          f"necesarios: {self.numRobots} | "
                          f"ArUco visibles: {visible}")

                if len(foundRobots) == self.numRobots:
                    print(f"[Búsqueda] Todos los robots encontrados: {sorted(foundRobots)}")
                    robotsIPs = self.searchRobotsUdp()
                    self.processFoundRobots(foundRobots)

            elif len(robotsIPs) != 0:
                robotIP, isValidFrame = self.setupRobots(robotIP, robotsIPs, configuredRobots)
                if robotIP is not None:
                    setupRetries += 1
                    if setupRetries >= MAX_SETUP_RETRIES:
                        print("[Setup] Timeout detectando desplazamiento, reintentando giro...")
                        robotIP = None
                        setupRetries = 0
                else:
                    setupRetries = 0

            else:
                if not executed:
                    executed, resultsFrame = self.initializeVideoAndLogging(frame.shape[:2])
                    if not executed:
                        # initializeVideoAndLogging falló — reintentar en el próximo frame
                        continue

                timeLog = round((time.time() - self.startTime), 1)
                processingStart = time.time()
                if time.time() - lastStatusTime >= 2.0:
                    lastStatusTime = time.time()
                self.sendPositions(resultsFrame, timeLog)
                self.addFrame(frame, resultsFrame, timeLog)
                self.addTimeLog(timeLog, round(time.time() - processingStart, 4))


    # =========================================================================
    # ENVÍO DE POSICIONES Y LOGGING
    # =========================================================================

    def sendPositions(self, resultsFrame, timeLog):
        """
        Envía las posiciones de los robots y actualiza el registro de posiciones.

        Parámetros:
        - resultsFrame (ndarray): Frame de resultados donde se dibuja la posición.
        - timeLog (float): Tiempo actual desde el inicio.
        """
        for robot in self.robots.values():
            displacement = robot.getDisplacement()
            if displacement is not None:
                x, y, angle = robot.previousPose
                self._paintCoverage(resultsFrame, robot.id, x, y)

                instruction = f'POSE|{x}|{y}|{angle}'
                self.sendInstruction(robot.IP, [instruction], False)

                self.addPositionLog(timeLog, robot.id, robot.name,
                                    robot.previousPose, displacement)
        self._drawLegend(resultsFrame)


    def initializeVideoAndLogging(self, resolution):
        """
        Inicializa el proceso de grabación de video y el registro de posiciones.

        Parámetros:
        - resolution (tuple): (alto, ancho) del frame.

        Returns:
        - tuple: (True, resultsFrame)
        """
        h, w = resolution
        resultsFrame = np.full((h, w, 3), (255, 255, 255), dtype=np.uint8)

        self.cellPx = max(4, int(self.cellSizeMm / self.mmPixel))
        grid_h = (h + self.cellPx - 1) // self.cellPx
        grid_w = (w + self.cellPx - 1) // self.cellPx
        self.coverageGrid = np.full((grid_h, grid_w), -1, dtype=np.int8)

        # Dibujar líneas de grilla tenues para visualizar la cuadrícula vacía
        for gx in range(0, w, self.cellPx):
            cv2.line(resultsFrame, (gx, 0), (gx, h - 1), (220, 220, 220), 1)
        for gy in range(0, h, self.cellPx):
            cv2.line(resultsFrame, (0, gy), (w - 1, gy), (220, 220, 220), 1)

        self.frameQueue = multiprocessing.Queue()
        args = (
            resolution,
            self.numRobots,
            self.pathVideo,
            self.processInterval,
            self.frameQueue,
            self.debugResolution
        )
        self.videoProcess = multiprocessing.Process(target=videoWriter, args=args)
        self.videoProcess.start()
        self.startTime = time.time()
        self.createPositionLog()
        self.createTimeLog()

        for robot in self.robots.values():
            x, y, _ = robot.previousPose
            self._paintCoverage(resultsFrame, robot.id, x, y)
            self.addPositionLog(0, robot.id, robot.name, robot.previousPose, (0, 0))
        self._drawLegend(resultsFrame)

        self.inputInstruction()
        self.readUdpConnection()

        return True, resultsFrame


    def _robotColor(self, robot_id):
        """Retorna el color BGR asignado al robot según su ID."""
        return _ROBOT_COLORS_BGR[int(robot_id) % len(_ROBOT_COLORS_BGR)]

    def _paintCoverage(self, resultsFrame, robot_id, x_mm, y_mm):
        """
        Marca la celda de la grilla de cobertura que corresponde a (x_mm, y_mm)
        y la pinta con el color del robot. Si la celda ya pertenece a este robot,
        no hace nada (evita redibujados innecesarios).
        Coordenadas fuera de la grilla se ignoran silenciosamente.
        """
        if self.coverageGrid is None:
            return
        cell_x = int(x_mm / self.cellSizeMm)
        cell_y = int(y_mm / self.cellSizeMm)
        gh, gw = self.coverageGrid.shape
        if not (0 <= cell_x < gw and 0 <= cell_y < gh):
            return
        robot_idx = int(robot_id)
        if self.coverageGrid[cell_y, cell_x] == robot_idx:
            return
        self.coverageGrid[cell_y, cell_x] = robot_idx
        color = self._robotColor(robot_id)
        px0, py0 = cell_x * self.cellPx, cell_y * self.cellPx
        px1 = min(px0 + self.cellPx, resultsFrame.shape[1])
        py1 = min(py0 + self.cellPx, resultsFrame.shape[0])
        cv2.rectangle(resultsFrame, (px0, py0), (px1 - 1, py1 - 1), color, -1)
        cv2.rectangle(resultsFrame, (px0, py0), (px1 - 1, py1 - 1), (180, 180, 180), 1)

    def _drawLegend(self, resultsFrame):
        """Dibuja la leyenda de colores por robot en la esquina superior derecha del mapa."""
        patch, gap, margin = 18, 4, 8
        x0 = resultsFrame.shape[1] - 130
        y0 = margin
        for robot in self.robots.values():
            color = self._robotColor(robot.id)
            cv2.rectangle(resultsFrame, (x0, y0), (x0 + patch, y0 + patch), color, -1)
            cv2.rectangle(resultsFrame, (x0, y0), (x0 + patch, y0 + patch), (60, 60, 60), 1)
            label = f'R{robot.id}: {robot.name[:7]}'
            cv2.putText(resultsFrame, label, (x0 + patch + 4, y0 + patch - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (30, 30, 30), 1)
            y0 += patch + gap


    def cleanup(self):
        """
        Libera los recursos utilizados por la cámara y cierra las ventanas de OpenCV.
        """
        self.camera.release()

        if self.frameQueue is not None:
            self.frameQueue.put(None)
            time.sleep(0.2)
            while not self.frameQueue.empty():
                self.frameQueue.get()
            self.frameQueue.close()

        cv2.destroyAllWindows()


    def addFrame(self, frame, resultsFrame, timeLog):
        """
        Agrega un fotograma y el frame de resultados a la cola de procesamiento.

        Parámetros:
        - frame (ndarray): Frame actual de la cámara.
        - resultsFrame (ndarray): Frame de resultados.
        - timeLog (float): Tiempo transcurrido en segundos.
        """
        cv2.putText(frame, f'Time: {timeLog} s', (2, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 200), 1, cv2.LINE_AA)
        # Enviar a la GUI si está disponible; siempre encolar para grabación en disco
        if self.gui is not None:
            self.gui.frameSignal.emit(frame.copy(), resultsFrame.copy())
        if self.frameQueue is not None:
            self.frameQueue.put([frame, resultsFrame])


    # =========================================================================
    # LOGGING
    # =========================================================================

    def createTimeLog(self):
        """Crea un archivo CSV de registro de tiempos de procesamiento."""
        currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
        logName = f'Time_Log_{currentTime}_Robots_{self.numRobots}.csv'
        os.makedirs('Logs', exist_ok=True)
        self.pathTimeLogs = os.path.join('Logs', logName)
        header = ['time', 'processingTime']
        with open(self.pathTimeLogs, 'w', newline='') as f:
            csv.writer(f).writerow(header)


    def addTimeLog(self, timeLog, processingTime):
        """Agrega una entrada al registro de tiempo de procesamiento."""
        row = [timeLog, processingTime]
        with open(self.pathTimeLogs, 'a', newline='') as f:
            csv.writer(f).writerow(row)


    def createPositionLog(self):
        """Crea un archivo CSV de registro de posiciones de robots."""
        currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
        logName = f'Position_Log_{currentTime}_Robots_{self.numRobots}.csv'
        self.pathPositionLogs = os.path.join(self.pathPositionLogs, logName)
        header = ['time', 'idrobot', 'robot', 'x', 'y', 'angle',
                  'linearDisplacement', 'angularDisplacement']
        with open(self.pathPositionLogs, 'w', newline='') as f:
            csv.writer(f).writerow(header)


    def addPositionLog(self, timeLog, id, name, position, displacement):
        """Agrega una entrada al registro de posiciones de los robots."""
        row = [timeLog, id, name, *position, *displacement]
        with open(self.pathPositionLogs, 'a', newline='') as f:
            csv.writer(f).writerow(row)


    def createConcoleLog(self):
        """Crea un archivo CSV de registro de mensajes UDP recibidos."""
        currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
        logName = f'Console_Log_{currentTime}_Robots_{self.numRobots}.csv'
        self.pathConsolelog = os.path.join(self.pathConsolelog, logName)
        header = ['time', 'idrobot', 'robot', 'message']
        with open(self.pathConsolelog, 'w', newline='') as f:
            csv.writer(f).writerow(header)


    def addConcoleLog(self, timeLog, id, name, message):
        """Agrega una entrada al registro de la consola UDP."""
        row = [timeLog, id, name, message]
        with open(self.pathConsolelog, 'a', newline='') as f:
            csv.writer(f).writerow(row)


    # =========================================================================
    # COMUNICACIÓN UDP
    # =========================================================================

    def searchRobotsUdp(self):
        """
        Busca robots en la red mediante transmisión UDP.

        Retorna:
        - list: Lista de IPs de robots encontrados.
        """
        instruction = [f'CONFIG|START|{self.broadcastIP}']
        self.sendInstructionBroadcast(instruction)
        robotsIPs = set()

        self.sock.settimeout(2.0)
        while len(robotsIPs) < self.numRobots:
            try:
                data, addr = self.sock.recvfrom(1024)
                if addr[0] != self.baseIP:
                    print(f"Respuesta de {addr[0]}: {data.decode()}")
                    robotsIPs.add(addr[0])
            except socket.timeout:
                try:
                    self.sendInstructionBroadcast(instruction)
                except OSError as e:
                    print(f"⚠ Error al reenviar broadcast: {e}")
                    break

        return list(robotsIPs)


    def sendInstructionBroadcast(self, instructions):
        """Envía instrucciones a todos los robots por broadcast."""
        for instruction in instructions:
            self.sock.sendto(instruction.encode(), (self.broadcastIP, self.port))
            print(f"(Broadcast) Mensaje enviado: {instruction}")


    @runOnThread
    def sendInstruction(self, ip, instructions, printing):
        """
        Envía instrucciones a un robot específico por IP.

        Parámetros:
        - ip (str): Dirección IP del robot.
        - instructions (list): Lista de instrucciones a enviar.
        - printing (bool): Si True, imprime confirmación en consola.
        """
        for instruction in instructions:
            self.sock.sendto(instruction.encode(), (ip, self.port))
            name = next((robot.name for robot in self.robots.values() if robot.IP == ip), ip)
            if printing:
                self.log(f'Mensaje enviado a {name}: {instruction}')


    @runOnThread
    def readUdpConnection(self):
        """
        Escucha y procesa mensajes recibidos a través de la conexión UDP.
        """
        self.createConcoleLog()
        self.sock.settimeout(None)

        while True:
            data, addr = self.sock.recvfrom(1024)
            ip = addr[0]

            if ip != self.baseIP:
                message = data.decode()
                timeLog = round(time.time() - self.startTime, 1)

                robotFound = False
                for robot in self.robots.values():
                    if robot.IP == ip:
                        name, id = robot.name, robot.id
                        robotFound = True
                        break

                if not robotFound:
                    name, id = ip, "-1"

                self.addConcoleLog(timeLog, id, name, message)

                parts = message.split('|')
                command = parts[0]

                if command == 'REQUEST_POSITION':
                    if robotFound:
                        self.sendPositionToRobot(ip, id)
                    if len(parts) >= 5 and parts[1] == 'BUG2':
                        self.log(f'Solicitud GT {name}: {parts[2]} paso={parts[3]} dist={parts[4]}mm')
                    else:
                        self.log(f'Solicitud de posición de {name}')

                elif command == 'LEADER_POSITION':
                    if len(parts) >= 5:
                        leaderID = parts[1]
                        leaderX, leaderY, leaderAngle = float(parts[2]), float(parts[3]), float(parts[4])
                        self.updateRobotPosition(leaderID, leaderX, leaderY, leaderAngle)
                        self.log(f'Posición de líder {leaderID}: ({leaderX},{leaderY}) {leaderAngle}°')

                elif command == 'CHECK_OBSTACLE':
                    continue

                else:
                    self.log(f'Mensaje de {name}: {message}')


    @runOnThread
    def sendPositionToRobot(self, robotIP, robotID):
        """
        Envía la posición actual de un robot específico vía UDP.

        Parámetros:
        - robotIP (str): IP del robot.
        - robotID (str): ID del robot.
        """
        if robotID not in self.robots:
            print(f"Robot {robotID} no encontrado")
            return

        robot = self.robots[robotID]
        x, y, angle = robot.getPose()
        if x == -1 and y == -1 and angle == -1:
            print(f"Posición no disponible para robot {robotID} (no visible en ArUco)")
            return

        message = f'POSITION_RESPONSE|{x}|{y}|{angle}'
        self.sendInstruction(robotIP, [message], False)
        self.log(f'Posición enviada a {robot.name}: x={x}, y={y}, angle={angle}')


    # =========================================================================
    # CONGREGACIÓN Y NAVEGACIÓN GLOBAL
    # =========================================================================

    def startCongregation(self, leaderID):
        """
        Inicia congregación con un líder designado.
        Asigna un slot de estacionamiento único a cada seguidor (opción B: parking spot).
        """
        if leaderID not in self.robots:
            print(f"Error: Robot líder {leaderID} no encontrado")
            return

        self.congregationActive = True
        self.leaderID = leaderID

        # Seguidores ordenados por ID para asignación determinista de slots
        followers = sorted([rid for rid in self.robots if rid != leaderID])
        total = len(followers)

        # Enviar al líder (sin índice de follower — solo necesita saber que es líder)
        leader_cmd = f'CONGREGATION|{leaderID}|0|{total}'
        self.sendInstruction(self.robots[leaderID].IP, [leader_cmd], False)

        # Enviar a cada seguidor su slot individual
        for idx, rid in enumerate(followers):
            cmd = f'CONGREGATION|{leaderID}|{idx}|{total}'
            self.sendInstruction(self.robots[rid].IP, [cmd], False)
            print(f"  Seguidor {self.robots[rid].name}: slot {idx}/{total}")

        print(f"Congregación iniciada. Líder: {self.robots[leaderID].name}, {total} seguidor(es)")


    def sendToGlobalPosition(self, robotID, targetX, targetY):
        """
        Envía un robot a una posición global específica.

        Parámetros:
        - robotID (str): ID del robot.
        - targetX (float): Coordenada X objetivo en mm.
        - targetY (float): Coordenada Y objetivo en mm.
        """
        if robotID not in self.robots:
            print(f"Error: Robot {robotID} no encontrado")
            return

        robot = self.robots[robotID]
        instruction = f'POSITIONGT|{targetX}|{targetY}'
        self.sendInstruction(robot.IP, [instruction], True)
        print(f"Robot {robot.name} enviado a posición: x={targetX}, y={targetY}")


    def updateRobotPosition(self, robotID, x, y, angle):
        """
        Actualiza la posición de un robot en el registro interno.

        Parámetros:
        - robotID (str): ID del robot.
        - x, y (float): Coordenadas en mm.
        - angle (float): Ángulo en grados.
        """
        self.robotPositions[robotID] = {
            'x': x, 'y': y, 'angle': angle,
            'timestamp': time.time()
        }


    def getDistanceBetweenRobots(self, robotID1, robotID2):
        """
        Calcula la distancia euclidiana entre dos robots.

        Retorna:
        - float: Distancia en mm, o -1 si algún robot no existe o no tiene pose.
        """
        if robotID1 not in self.robots or robotID2 not in self.robots:
            return -1

        x1, y1, _ = self.robots[robotID1].previousPose
        x2, y2, _ = self.robots[robotID2].previousPose

        if x1 == -1 or x2 == -1:
            return -1

        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


    def isCongregationComplete(self, threshold=100):
        """
        Verifica si todos los robots están cerca del líder.

        Parámetros:
        - threshold (float): Distancia máxima en mm para considerar "cerca".

        Retorna:
        - bool: True si todos están dentro del umbral respecto al líder.
        """
        if not self.congregationActive or self.leaderID is None:
            return False

        for robotID in self.robots:
            if robotID == self.leaderID:
                continue
            distance = self.getDistanceBetweenRobots(self.leaderID, robotID)
            if distance == -1 or distance > threshold:
                return False

        return True


    # =========================================================================
    # ENTRADA DE USUARIO
    # =========================================================================

    @runOnThread
    def inputInstruction(self):
        """
        Maneja la entrada de instrucciones desde la consola en tiempo real.

        Formato: 'robotId.instrucción' o comandos especiales:
            BROADCAST.instrucción
            CONGREGATION.leaderID
            GOTO.robotID x y
            STATUS.(cualquier cosa)
            BREAK
        """
        while True:
            instructionRaw = input('').strip()
            if instructionRaw == 'BREAK':
                break

            try:
                robotId, instruction = map(str.strip, instructionRaw.split('.', 1))
            except ValueError:
                print("Formato inválido. Use 'robotId.instrucción'")
                continue

            if robotId == 'BROADCAST':
                self.sendInstructionBroadcast([instruction])
            elif robotId in self.robots:
                robotIP = self.robots[robotId].IP
                self.sendInstruction(robotIP, [instruction], True)
            elif robotId == 'CONGREGATION':
                self.startCongregation(instruction)
            elif robotId == 'GOTO':
                parts = instruction.split()
                if len(parts) == 3:
                    targetRobotID = parts[0]
                    targetX = float(parts[1])
                    targetY = float(parts[2])
                    self.sendToGlobalPosition(targetRobotID, targetX, targetY)
                else:
                    print("Formato: GOTO.robotID x y")
            elif robotId == 'STATUS':
                print(f"Detecciones ArUco activas: {list(self.currentArucoDetections.keys())}")
                for rid, robot in self.robots.items():
                    x, y, angle = robot.getPose()
                    if x != -1:
                        print(f"  Robot {rid} ({robot.name}): x={x}, y={y}, angle={angle}°")
                    else:
                        print(f"  Robot {rid} ({robot.name}): no visible")
                if self.congregationActive:
                    print(f"Congregación activa. Líder: {self.leaderID}")
                    print(f"Completa: {self.isCongregationComplete()}")
            else:
                print(f"Robot ID '{robotId}' no encontrado.")

        self.threadInputAlive = False


# =============================================================================
# PUNTO DE ENTRADA
# =============================================================================

base = Base()


def main():
    configurationFilePath = 'configSystem.json'
    base.numRobots = int(input('Cantidad de robots en la prueba: '))
    base.readConfigFile(configurationFilePath)
    base.cameraProcessing()


if __name__ == '__main__':
    main()