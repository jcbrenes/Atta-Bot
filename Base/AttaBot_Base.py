import os
os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')  # evita bloqueo de imshow en Wayland/Gnome
import cv2, json, math, time, socket, threading, csv, multiprocessing, threading, platform
import numpy as np
import readline
from datetime import datetime


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

        resizeResultFrame = cv2.resize(resultsFrame, debugResolution, interpolation=cv2.INTER_AREA)
        resizeFrame = cv2.resize(frame, debugResolution, interpolation=cv2.INTER_AREA)
        resizeResults = cv2.vconcat([resizeFrame, resizeResultFrame])
        cv2.imshow('Recorrido robots', resizeResults)

        results = cv2.vconcat([frame, resultsFrame])
        video.write(results)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

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
            float: El desplazamiento angular en grados, redondeado a un decimal.
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
        base.sendInstruction(ip, [f'CONFIG|{self.id}'], False)



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
        self.currentArucoDetections = {}  # {robot_id: (x_mm, y_mm, angle_deg)}
        self.bigCircleRadius = 10         # radio visual en el resultsFrame (px)


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
        # Detectar en media resolución (4x más rápido), escalar esquinas de vuelta
        small = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        corners, ids, _ = self.arucoDetector.detectMarkers(small)
        if corners:
            corners = [c * 2.0 for c in corners]

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

        for i, marker_id in enumerate(ids.flatten()):
            imagePoints = corners[i][0].astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(
                objectPoints,
                imagePoints,
                self.cameraMatriz,
                self.distance,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not success:
                continue

            # Posición en mm (tvec está en metros), relativa al origen configurado
            x_mm = round(float(tvec[0][0]) * 1000.0 - self.originXmm, 1)
            y_mm = round(float(tvec[1][0]) * 1000.0 - self.originYmm, 1)

            # Ángulo: extraer heading del eje X del marker proyectado en el plano XY
            rotMatrix, _ = cv2.Rodrigues(rvec)
            angle_rad = np.arctan2(rotMatrix[1][0], rotMatrix[0][0])
            angle_deg = round(float(np.degrees(angle_rad) % 360), 1)

            detectedPoses[str(marker_id)] = (x_mm, y_mm, angle_deg)

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

            displacement = robot.getDisplacement()
            if displacement is not None:
                _, angularDisplacement = displacement
                if abs(angularDisplacement) >= 45:
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

        # Radio visual para el resultsFrame (proporcional al tamaño del marker en px)
        self.bigCircleRadius = max(6, int((self.markerSizeMm / self.mmPixel) * 0.5))

        # Inicializar detector ArUco
        # DICT_4X4_50: markers 4x4 bits, 50 IDs disponibles — robusto y compacto
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()

        arucoParams.adaptiveThreshWinSizeMin = 3
        arucoParams.adaptiveThreshWinSizeMax = 53    # 14 iteraciones (equilibrio velocidad/robustez)
        arucoParams.adaptiveThreshWinSizeStep = 4
        arucoParams.adaptiveThreshConstant = 5       # más sensible que default (7) en zonas oscuras
        arucoParams.minMarkerPerimeterRate = 0.003
        arucoParams.maxMarkerPerimeterRate = 4.0
        arucoParams.polygonalApproxAccuracyRate = 0.08
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

        # Detección ArUco — actualiza el diccionario accedido por Robot.getPose()
        self.currentArucoDetections = self.detectArucoMarkers(frame)

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
                self.createCircle(resultsFrame, x, y)

                instruction = f'POSE|{x}|{y}|{angle}'
                self.sendInstruction(robot.IP, [instruction], False)

                self.addPositionLog(timeLog, robot.id, robot.name,
                                    robot.previousPose, displacement)


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
            self.createCircle(resultsFrame, x, y)
            self.addPositionLog(0, robot.id, robot.name, robot.previousPose, (0, 0))

        self.inputInstruction()
        self.readUdpConnection()

        return True, resultsFrame


    def createCircle(self, resultsFrame, x, y):
        """
        Dibuja un círculo en el frame de resultados representando la posición de un robot.

        Parámetros:
        - resultsFrame (ndarray): Frame de resultados.
        - x (float): Coordenada x en mm.
        - y (float): Coordenada y en mm.
        """
        printX = int(x / self.mmPixel)
        printY = int(y / self.mmPixel)
        cv2.circle(resultsFrame, [printX, printY], self.bigCircleRadius, (255, 0, 0), -1)


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
        text = f'Time: {timeLog} s'
        position = (2, 26)
        cv2.putText(frame, text, position, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 200), 1, cv2.LINE_AA)
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
                print(f"Mensaje enviado a {name}: {instruction}")


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
                        b_state = parts[2]
                        b_steps = parts[3]
                        b_dist  = parts[4]
                        print(f"[Bug2 {b_state} paso={b_steps} dist={b_dist}mm] {name}")
                    else:
                        print(f"Solicitud de posición de {name}")

                elif command == 'LEADER_POSITION':
                    if len(parts) >= 5:
                        leaderID = parts[1]
                        leaderX = float(parts[2])
                        leaderY = float(parts[3])
                        leaderAngle = float(parts[4])
                        self.updateRobotPosition(leaderID, leaderX, leaderY, leaderAngle)
                        print(f"Posición de líder {leaderID}: x={leaderX}, y={leaderY}, angle={leaderAngle}")

                elif command == 'CHECK_OBSTACLE':
                    continue

                else:
                    print(f"Mensaje de {name}: {message}")


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
        print(f"Posición enviada a {robot.name}: x={x}, y={y}, angle={angle}")


    # =========================================================================
    # CONGREGACIÓN Y NAVEGACIÓN GLOBAL
    # =========================================================================

    def startCongregation(self, leaderID):
        """
        Inicia el proceso de congregación con un líder designado.

        Parámetros:
        - leaderID (str): ID del robot líder.
        """
        if leaderID not in self.robots:
            print(f"Error: Robot líder {leaderID} no encontrado")
            return

        self.congregationActive = True
        self.leaderID = leaderID

        instruction = f'CONGREGATION|{leaderID}'
        self.sendInstructionBroadcast([instruction])
        print(f"Congregación iniciada. Líder: {self.robots[leaderID].name}")


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