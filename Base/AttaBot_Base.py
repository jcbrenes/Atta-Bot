import cv2, json, math, time, socket, threading, os, csv, multiprocessing, threading
import numpy as np
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
    mostrando además el video en una ventana .

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
      


class Color(object):
    """
    La clase Color representa un color específico con límites de umbral para segmentación de imágenes,
    configurando el tamaño de los círculos pequeños y grandes en función de los parámetros de configuración
    y proporcionando métodos para procesar cuadros de video en busca de estos círculos.

    Parámetros de inicialización:
    - name (str): Nombre del color (utilizado para ventanas de depuración).
    - configColor (dict): Configuración de límites de color con 'light' y 'dark' para la segmentación.
    - configLimits (dict): Límites de configuración para el tamaño de círculos y margen de error.
    - mmPixel (float): Escala en mm/píxel para calcular el área y posición.

    Atributos:
    - light (tuple): Límite bajo del color en HSV.
    - dark (tuple): Límite alto del color en HSV.
    - contours (dict): Contornos de los círculos encontrados, separados por tipo ('small' y 'big').
    - contourAreas (dict): Áreas de los contornos encontrados.
    - centroids (dict): Coordenadas de los centroides de cada contorno.
    - mask (list): Máscara binaria del color en el cuadro procesado.
    - processedMask (list): Máscara procesada para la detección de contornos.
    """

    def __init__(self, name, configColor, configLimits, mmPixel):
        self.name = name
        self.nameWindowDebug = f'Debug Color: {self.name}'
        self.smallCircleMin = int
        self.smallCircleMax = int
        self.bigCircleMin = int
        self.bigCircleMax = int
        self.smallRadius = int
        self.bigRadius = int
        self.mmPixel = mmPixel
        self.light = []
        self.dark = []
        self.contours = {}
        self.contourAreas = {}
        self.centroids = {}
        self.mask = []
        self.processedMask = []
        self.initColor(configColor, configLimits)


    def initColor(self, configColor, configLimits):
        """
        Configura los límites de color y el tamaño de los círculos en base a la configuración.

        Parámetros:
        - configColor (dict): Configuración de los límites de color.
        - configLimits (dict): Límites de tamaño de los círculos y margen de error.

        Retorna:
        None
        """
        self.light = tuple(configColor['light'])
        self.dark = tuple(configColor['dark'])

        self.bigRadius = configLimits['big_circle']
        self.smallRadius = configLimits['small_circle']
        marginFactor = float(configLimits['margin_percentage']) / 100

        smallArea = math.pi * pow(self.smallRadius, 2)
        bigArea = math.pi * pow(self.bigRadius, 2)

        self.smallCircleMin = int(smallArea * (1 - marginFactor))
        self.smallCircleMax = int(smallArea * (1 + marginFactor))
        self.bigCircleMin = int(bigArea * (1 - marginFactor))
        self.bigCircleMax = int(bigArea * (1 + marginFactor))
    

    def contourFilter(self, contoursRaw):
        """
        Filtra contornos basándose en su área y clasifica los contornos como 'small' o 'big' 
        según las áreas configuradas para círculos pequeños y grandes.
        Cada contorno se ajusta mediante convexHull para mejorar su precisión.

        Parámetros:
        - contoursRaw (list): Lista de contornos en bruto obtenidos de una imagen.

        Retorna:
        - contours (dict): Diccionario con los contornos filtrados, clasificados en 'small' y 'big'.
        - contourAreas (dict): Diccionario con las áreas de cada contorno en 'small' y 'big'.
        """
        contours = {'small': [], 'big': []}
        contourAreas = {'small': [], 'big': []}

        smallMin, smallMax = self.smallCircleMin, self.smallCircleMax
        bigMin, bigMax = self.bigCircleMin, self.bigCircleMax

        for contour in contoursRaw:
            contour = cv2.convexHull(contour)
            area = int(cv2.contourArea(contour) * (pow(self.mmPixel, 2)))

            if smallMin <= area <= smallMax:
                contours['small'].append(contour)
                contourAreas['small'].append(area)
            elif bigMin <= area <= bigMax:
                contours['big'].append(contour)
                contourAreas['big'].append(area)

        return contours, contourAreas


    def centroidsContours(self):
        """
        Calcula los centroides de los contornos almacenados y los clasifica en dos categorías: 
        'small' y 'big' en función de sus áreas.

        Retorna:
        - centroids (dict): Diccionario con los centroides de los contornos clasificados como 
        'small' y 'big', en formato (cx, cy), donde cx y cy son las coordenadas en milímetros 
        ajustadas con el factor mmPixel.
        """
        centroids = {'small': [], 'big': []}
        for circleType, contourList in self.contours.items():
            for contour in contourList:
                moments = cv2.moments(contour)
                # Calcula el centroide
                # https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
                cx = round((moments['m10'] / moments['m00']) * self.mmPixel, 1)
                cy = round((moments['m01'] / moments['m00']) * self.mmPixel, 1)
                centroids[circleType].append((cx, cy))

        return centroids


    def processFrame(self, frame):
        """
        Procesa un frame de video para detectar áreas que caen dentro de los límites de color especificados.

        Parámetros:
        - frame (numpy.ndarray): Un frame de imagen en formato HSV que se procesará para 
        detectar los colores definidos por los atributos 'light' y 'dark' de la clase.

        Este método realiza las siguientes operaciones:
        1. Crea una máscara binaria que destaca los píxeles dentro del rango de color especificado.
        2. Aplica una transformación morfológica de apertura para reducir el ruido y mejorar 
        la uniformidad de los contornos detectados.
        3. Utiliza un filtro de mediana para suavizar la máscara procesada.
        4. Dilata la máscara procesada para aumentar el tamaño de las regiones detectadas.
        5. Busca los contornos en la máscara procesada y los clasifica utilizando el método 
        `contourFilter`, además de calcular los centroides con `centroidsContours`.
        """
        self.mask = cv2.inRange(frame, self.light, self.dark)

        # Se utiliza una transformación morfológica de dilatación de la imagen para hacer los contornos más uniformes
        # Se crea el kernel, entre mayor sea más "grosera" es la dilatación
        kernel = np.ones((4, 4),np.uint8)
        processedMask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, kernel)
        processedMask = cv2.medianBlur(processedMask, 5)
        self.processedMask = cv2.dilate(processedMask, kernel, iterations=1)

        # Busca los contornos en cada una de las imágenes filtradas
        # Se aproximan por medio de una compresión horizontal, vertical y diagonal
        # https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0
        contours, _ = cv2.findContours(self.processedMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.contours, self.contourAreas = self.contourFilter(contours)
        self.centroids = self.centroidsContours()


    def debug(self):
        """
        Muestra una ventana de depuración con las máscaras y contornos procesados.

        No se requieren parámetros y no se devuelve ningún valor. La función se utiliza para visualizar los resultados de 
        la detección de contornos y centroids, facilitando la depuración del procesamiento de imágenes.
        """
        mask = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)
        processedMask = cv2.cvtColor(self.processedMask, cv2.COLOR_GRAY2BGR)

        borderSize = 4
        mask = cv2.copyMakeBorder(mask, borderSize, borderSize, borderSize, borderSize, cv2.BORDER_CONSTANT, value=(0, 0, 255))
        processedMask = cv2.copyMakeBorder(processedMask, borderSize, borderSize, borderSize, borderSize, cv2.BORDER_CONSTANT, value=(0, 0, 255))

        for circleType, contourList in self.contours.items(): 
            for i, contour in enumerate(contourList):
                cv2.drawContours(processedMask, [contour], 0, (255, 0, 0), 2)
                x = int(self.centroids[circleType][i][0] / self.mmPixel)
                y = int(self.centroids[circleType][i][1] / self.mmPixel)
                cv2.circle(processedMask, (x, y), 1, (0, 0, 255), 6)

                mmX = round(x * self.mmPixel)
                mmY = round(y * self.mmPixel)
                
                text1 = f'[{mmX},{mmY}]'
                text2 = f'Area[mm^2]: {self.contourAreas[circleType][i]}'
                radius = int(self.bigRadius / self.mmPixel * 2.2)
                position1 = (x - radius  , y + radius)
                position2 = (x - radius, y + radius + 40)
                cv2.putText(processedMask, text1, position1, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(processedMask, text2, position2, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1, cv2.LINE_AA)

        mask = cv2.resize(mask, base.debugResolution, interpolation=cv2.INTER_AREA)
        processedMask = cv2.resize(processedMask, base.debugResolution, interpolation=cv2.INTER_AREA)
        debug = cv2.vconcat([mask, processedMask])
        cv2.imshow(self.nameWindowDebug, debug)



class Robot(object):
    """
    Clase que representa un robot en un sistema de enjambre.

    Atributos:
        id (int): Identificador único del robot.
        name (str): Nombre del robot.
        colors (list): Lista de colores asociados al robot.
        IP (str): Dirección IP del robot para la comunicación.
        previousPose (list): Última posición conocida del robot en formato [x, y, a].
    """

    def __init__(self, id, configRobot):
        self.id = id
        self.name = ''
        self.colors = []
        self.IP = ''
        self.previousPose = []
        self.initRobot(configRobot)


    def getAngle(self, smallCircle, bigCircle):
        """
        Calcula el ángulo entre los centros de dos círculos (grande y pequeño)
        respecto a la horizontal.

        El ángulo se calcula en grados, tomando como referencia la línea que se forma
        entre los dos círculos, donde los ángulos positivos se consideran en sentido horario.

        Args:
            smallCircle (tuple): Coordenadas (x, y) del círculo pequeño.
            bigCircle (tuple): Coordenadas (x, y) del círculo grande.

        Returns:
            float: El ángulo en grados entre la línea que se forma entre el círculo grande
                y el pequeño, en relación a la horizontal.
        """
        x1, y1 = bigCircle
        x2, y2 = smallCircle

        # Calculo de cita respecto a la horizontal, tomando ángulos positivos en sentido horario
        angle = round(float(np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 360), 1)
            
        return angle


    def getPose(self):
        """
        Obtiene la pose actual del robot utilizando las posiciones de los círculos
        grande y pequeño.

        Se busca el círculo grande y se compara con todos los círculos pequeños.
        Si la distancia entre un círculo grande y un pequeño está dentro de un umbral
        definido, se calcula la posición (x, y) y el ángulo (a) del robot.

        Returns:
            tuple: Un tuple que contiene la posición (x, y) y el ángulo (a) del robot.
                Si no se encuentra una pose válida, devuelve (-1, -1, -1).
        """
        bigCircles = base.frameColors[self.colors[0]].centroids['big']
        smallCircles = base.frameColors[self.colors[1]].centroids['small']

        for bigCircle in bigCircles:
            for smallCircle in smallCircles:
                distance = math.dist(bigCircle, smallCircle)
                if distance <= base.mmCenterDistance:
                    x, y = bigCircle
                    a = self.getAngle(smallCircle, bigCircle)
    
                    return (x, y, a)
        return (-1, -1, -1)


    def initRobot(self, configRobot):
        """
        Inicializa el robot configurando su nombre y colores, y establece
        su pose inicial.

        Este método toma un diccionario de configuración del robot y asigna
        el nombre y los colores del robot según su ID. Luego, obtiene y
        guarda la pose actual del robot.

        Args:
            configRobot (dict): Un diccionario que contiene la configuración
                                de los robots.
        """
        self.name = configRobot[self.id]['name']
        self.colors = configRobot[self.id]['colors']
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

        linearDisplacement = self.linearDisplacement(x1, y1, a1, x2, y2)
        angularDisplacement = self.angularDisplacement(a1, a2)

        if abs(angularDisplacement) >= 4 or abs(linearDisplacement) >= 4:
            self.previousPose = currentPose
            return linearDisplacement, angularDisplacement
        
        return None


    def angularDisplacement(self, a1, a2):
        """
        Calcula el desplazamiento angular entre dos ángulos.

        Este método determina la diferencia angular entre dos ángulos dados, 
        asegurándose de que el resultado se mantenga en el rango de -180 a 180 
        grados. Esto permite interpretar correctamente la dirección de rotación.

        Args:
            a1 (float): El ángulo inicial en grados.
            a2 (float): El ángulo final en grados.

        Returns:
            float: El desplazamiento angular en grados, redondeado a un decimal.
                Un valor positivo indica un desplazamiento en sentido horario,
                mientras que un valor negativo indica un desplazamiento en sentido 
                antihorario.
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

        Este método utiliza el ángulo de la primera posición para determinar si el 
        desplazamiento es en la misma dirección que el ángulo o en la dirección opuesta. 
        Si el desplazamiento es en la dirección opuesta, se devuelve un valor negativo.

        Args:
            x1 (float): Coordenada x de la primera posición.
            y1 (float): Coordenada y de la primera posición.
            a1 (float): Ángulo en grados de la primera posición, donde 0 grados 
                        representa la dirección positiva del eje x.
            x2 (float): Coordenada x de la segunda posición.
            y2 (float): Coordenada y de la segunda posición.

        Returns:
            float: El desplazamiento lineal entre las dos posiciones, redondeado a 
                un decimal. Un valor positivo indica un desplazamiento en la 
                dirección del ángulo, mientras que un valor negativo indica un 
                desplazamiento en la dirección opuesta.
        """
        a1Rad = math.radians(a1)
        dx = x2 - x1
        dy = y2 - y1

        # Producto escalar entre el desplazamiento y el ángulo
        dotProduct = dx * math.cos(a1Rad) + dy * math.sin(a1Rad)
        displacement = round(math.dist((x1, y1), (x2, y2)), 1)

        return displacement if dotProduct >= 0 else -displacement


    def setupIP(self, ip):
        """
        Configura la dirección IP del robot y envía una instrucción de configuración 
        al robot.

        Este método asigna la dirección IP proporcionada a la instancia del robot 
        y envía un comando de configuración al robot.

        Args:
            ip (str): La dirección IP que se asignará al robot. Debe estar en formato 
                    de dirección IPv4 (ej. '192.168.1.1').
        
        Returns:
            None
        """
        self.IP = ip
        base.sendInstruction(ip, [f'CONFIG|{self.id}'], False)



class Base(object):
    """
    Clase Base para la gestión de un sistema de robots de enjambre.

    Esta clase encapsula la configuración y las funcionalidades necesarias para la 
    operación de un sistema de visión y control de robots. Proporciona métodos 
    para la configuración de la cámara, la comunicación a través de UDP, y el 
    procesamiento de imágenes y colores.

    Atributos:
        frameColors (dict): Diccionario que almacena los colores de los frame detectados.
        robots (dict): Diccionario que contiene la configuración y estado de los robots.
        robotsConfig (dict): Configuración de los robots leída desde un archivo.
        numRobots (int): Número de robots en el sistema.
        debug (bool): Indica si el modo de depuración está habilitado.
        camera (cv2.VideoCapture): Objeto de captura de video para la cámara.
        debugResolution (tuple): Resolución para la visualización de depuración.
        mmPixel (float): Escala de milímetros por píxel para la conversión de unidades.
        mmCenterDistance (int): Distancia en milímetros entre los centros de los círculos.
        bigCircleRadius (int): Radio del círculo grande utilizado en el procesamiento de imágenes.
        processInterval (float): Intervalo de procesamiento entre frames.
        startTime (float): Tiempo de inicio de la prueba.
        cameraMatriz (list): Matriz de calibración de la cámara.
        distance (list): Distancia utilizada para la calibración de la cámara.
        sock (socket.socket): Socket UDP para la comunicación.
        baseIP (str): Dirección IP de la base.
        broadcastIP (str): Dirección IP de broadcast para la comunicación UDP.
        port (int): Puerto para la comunicación UDP.
        threadInputAlive (bool): Indica si el hilo de entrada está activo.
        pathVideo (str): Ruta para guardar el videos de la prueba.
        pathPositionLogs (str): Ruta para guardar los registros de posición.
        pathConsolelog (str): Ruta para guardar los registros de consola.
        cameraResolution (list): Resolución de la cámara.
        newCameraMatriz (np.ndarray): Nueva matriz de cámara optimizada.
        roi (tuple): Región de interés para la corrección de la cámara.
        frameQueue (multiprocessing.Queue): Cola para manejar los frames procesados.
        videoProcess (multiprocessing.Process): Proceso para la grabación de video.
    """

    def __init__(self):
        self.frameColors = {}
        self.robots = {}
        self.robotsConfig = {}
        self.numRobots = int
        self.debug = False
        self.camera = None
        self.debugResolution = tuple
        self.mmPixel = float
        self.mmCenterDistance = int
        self.bigCircleRadius = int
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


    def searchRobotsColor(self, robots):
        """
        Busca robots según sus colores asociados.

        Este método itera a través de un diccionario de robots, cada uno con identificadores y datos de color. 
        Para cada robot, se recuperan los centroides de los círculos de color grandes y pequeños asociados, 
        y luego se calcula la distancia entre cada par de círculos grandes y pequeños. Si la distancia está 
        dentro del umbral definido (`mmCenterDistance`), el identificador del robot se añade al conjunto 
        `foundRobots`, y los colores asociados se añaden al conjunto `foundColors`.

        Parámetros:
        robots (dict): Un diccionario donde las claves son identificadores de robots y los valores son 
                        diccionarios que contienen información de los colores del robot.

        Retorna:
        tuple: Una tupla que contiene:
            - foundRobots (set): Un conjunto de identificadores de robots que se han encontrado dentro del umbral de distancia.
            - foundColors (set): Un conjunto de colores asociados con los robots encontrados.
        """
        foundRobots = set()
        foundColors = set()

        for identifier, data in robots.items():
            colors = data['colors']
            bigCircles = self.frameColors[colors[0]].centroids['big']
            smallCircles = self.frameColors[colors[1]].centroids['small']

            for bigCircle in bigCircles:
                for smallCircle in smallCircles:
                    distance = math.dist(bigCircle, smallCircle)
                    if distance <= self.mmCenterDistance:
                        foundRobots.add(identifier)
                        foundColors.update(colors)
                        break
        
        return foundRobots, foundColors


    def readConfigFile(self, filePath):
        """
        Lee un archivo de configuración y aplica las configuraciones del sistema.

        Este método abre un archivo de configuración en formato JSON, carga su contenido y aplica 
        las configuraciones necesarias para el sistema de visión, comunicación UDP y configuraciones 
        generales. Además, inicializa la configuración de los robots y los colores asociados.

        Parámetros:
        filePath (str): La ruta del archivo de configuración que se debe leer.

        Excepciones:
        Raises:
            FileNotFoundError: Si el archivo especificado no se encuentra.
            json.JSONDecodeError: Si el archivo no se puede decodificar como JSON.
        """
        with open(filePath, 'r') as file:
            configuration = json.load(file)

        self.configVisionSystem(configuration['vision_system'])
        self.configUdp(configuration['udp_communication'])
        self.generalConfig(configuration['general'])

        self.robotsConfig = configuration['robots']
        config = configuration['vision_system']['circles_radius']
        self.frameColors = {color: Color(color, configColor, config, self.mmPixel) for color, configColor in configuration['colors'].items()}


    def generalConfig(self, configuration):
        """
        Configura las opciones generales del sistema a partir de un diccionario de configuración.

        Este método permite habilitar el modo de depuración y establece las rutas para guardar videos, 
        registros de posición y registros de consola. Si alguna de las rutas especificadas no existe, 
        se intenta crear. En caso de que ocurra un error al crear las rutas, se lanza una excepción 
        con un mensaje de error detallado.

        Parámetros:
        configuration (dict): Un diccionario que contiene la configuración del sistema. Debe incluir las claves:
            - 'debug_enable' (bool): Indica si se debe habilitar el modo de depuración.
            - 'path_save_videos' (str): Ruta donde se guardarán los videos.
            - 'path_save_position_logs' (str): Ruta donde se guardarán los registros de posición.
            - 'path_save_console_logs' (str): Ruta donde se guardarán los registros de la consola.

        Excepciones:
        Raises:
            Exception: Si ocurre un error al intentar crear una ruta que no existe.
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

        Este método establece la dirección IP base, la dirección IP de broadcast y el puerto 
        para la comunicación UDP, ademas, crea un socket UDP. También habilita la opción de 
        broadcast en el socket.

        Parámetros:
        configuration (dict): Un diccionario que contiene la configuración de UDP, 
                            que incluye:
            - 'base_ip' (str): La dirección IP base del sistema.
            - 'broadcast_ip' (str): La dirección IP de broadcast.
            - 'port' (int): El puerto a utilizar para la comunicación UDP.
        """
        self.baseIP = configuration['base_ip']
        self.broadcastIP =  configuration['broadcast_ip']
        self.port = configuration['port']

        # Crear socket UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('', self.port))


    def configVisionSystem(self, configuration):
        """
        Configura el sistema de visión de la cámara.

        Este método establece la configuración de la cámara, carga las matrices de calibración, 
        calcula la nueva matriz de cámara óptima y define la resolución espacial de la imagen 
        en milímetros por píxel. También se configuran los parámetros relacionados con el 
        procesamiento de imágenes y el radio de los círculos grandes.

        Parámetros:
        configuration (dict): Un diccionario que contiene la configuración del sistema de 
                            visión, que incluye:
            - 'path_cameraMatrix' (str): La ruta al archivo que contiene la matriz de la cámara.
            - 'path_distance' (str): La ruta al archivo que contiene la distancia de calibración.
            - 'mmPixel' (str): La resolución espacial en mm/píxel, en formato 'numerador/denominador'.
            - 'distance_between_centers' (float): La distancia entre los centros de los círculos.
            - 'frame_processing_interval' (float): El intervalo de procesamiento de los fotogramas.
            - 'circles_radius' (dict): Un diccionario que contiene el radio de los círculos.
        """
        self.setCamera(configuration)

        # Carga los matrices de calibración
        self.cameraMatriz = np.loadtxt(configuration['path_cameraMatrix'], dtype=float)
        self.distance = np.loadtxt(configuration['path_distance'], dtype=float)
        h, w = self.cameraResolution
        self.newCameraMatriz, self.roi = cv2.getOptimalNewCameraMatrix(self.cameraMatriz, self.distance, (w,h), 1, (w,h))

        # Resolución espacial para imagen [mm/pixel]
        numerator, denominator = map(int, configuration['mmPixel'].split('/'))
        self.mmPixel = numerator / denominator
        self.mmCenterDistance = configuration['distance_between_centers']
        self.processInterval = configuration['frame_processing_interval']
        self.bigCircleRadius = int(configuration['circles_radius']['big_circle'] / self.mmPixel)


    def setCamera(self, configuration):
        """
        Inicializa la captura de video desde la cámara.

        Este método configura la cámara utilizando OpenCV, estableciendo la resolución de 
        la cámara y la resolución para depuración. Si la cámara no se puede abrir, se 
        imprime un mensaje de error y se finaliza el programa.

        Parámetros:
        configuration (dict): Un diccionario que contiene la configuración de la cámara, 
                            que incluye:
            - 'debug_resolution' (str): La resolución de depuración en formato 'ancho x alto'.
            - 'camera_resolution' (str): La resolución de la cámara en formato 'ancho x alto'.
        
        Raises:
            SystemExit: Si no se puede abrir la cámara.
        """

        # Inicializa la captura de video (0 es el ID de la cámara predeterminada)
        self.camera = cv2.VideoCapture(0, cv2.CAP_DSHOW) # CAP_DSHOW, CAP_MSMF
        self.debugResolution = tuple(map(int, configuration['debug_resolution'].split('x')))
        width, height = map(int, configuration['camera_resolution'].split('x'))
        self.cameraResolution = (height, width)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.camera.isOpened():
            print('Error: No se pudo abrir la cámara.')
            exit()


    def cameraCorrection(self, frame):
        """
        Desdistorsiona y recorta la imagen de la cámara.

        Este método corrige la distorsión de la imagen utilizando la matriz de calibración de 
        la cámara y los parámetros de distorsión, y luego recorta la imagen a la región de 
        interés (ROI) definida. Además, convierte la imagen desdistorsionada de BGR a HSV.

        Parámetros:
        frame (numpy.ndarray): La imagen de entrada en formato BGR que se va a corregir.

        Returns:
        tuple: Una tupla que contiene:
            - frame (numpy.ndarray): La imagen desdistorsionada y recortada en formato BGR.
            - frameHsv (numpy.ndarray): La imagen convertida a formato HSV.
        """
        x, y, w, h = self.roi
        frame = cv2.undistort(frame, self.cameraMatriz, self.distance, None, self.newCameraMatriz)[y:y+h, x:x+w]
        frameHsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        return frame, frameHsv


    def cameraDebug(self, frameHsv):
        """
        Muestra una ventana de depuración con la imagen en formato HSV.

        Este método redimensiona la imagen HSV recibida a una resolución de depuración 
        definida y la muestra en una ventana. Además, itera sobre los colores en el 
        sistema de visión y llama a su método de depuración.

        Parámetros:
        frameHsv (numpy.ndarray): La imagen en formato HSV que se va a mostrar.

        Returns:
        None
        """
        resizeFrameHsv = cv2.resize(frameHsv, self.debugResolution, interpolation=cv2.INTER_AREA)
        cv2.imshow('Debug Imagen HSV', resizeFrameHsv)

        for color in self.frameColors.values():
            color.debug()


    def processFoundRobots(self, foundRobots, foundColors):
        """
        Procesa los robots encontrados y actualiza los colores en el sistema de visión.

        Este método crea instancias de la clase `Robot` para cada robot encontrado,
        ordenándolos por su identificador. También elimina los colores que no fueron 
        detectados, cerrando las ventanas de depuración correspondientes si el modo 
        de depuración está habilitado.

        Parámetros:
        foundRobots (set): Un conjunto de identificadores de robots encontrados.
        foundColors (set): Un conjunto de colores que fueron detectados.

        Returns:
        None
        """
        self.robots = {robot: Robot(robot, self.robotsConfig) for robot in sorted(foundRobots, key=int)}

        removeColors = set(self.frameColors) - foundColors
        for color in removeColors:
            if self.debug:
                cv2.destroyWindow(self.frameColors[color].nameWindowDebug)
            self.frameColors.pop(color)


    def setupRobots(self, robotIP, robotsIPs):
        """
        Asocia una dirección IP a un robot en función de la detección del sistema de visión.

        Si no se proporciona una dirección IP se selecciona una de robotsIPs, despues se 
        gira el robot para identificar cuál se mueve y se le asigna la dirección IP 
        selecciona. 

        Parameters:
        - robotIP (str o None): Dirección IP actual del robot. Si es None, se
        seleccionara una nueva IP.
        - robotsIPs (list): Lista de direcciones IP disponibles para asignar a los robots.

        Returns:
        - tuple: Contiene una dirección IP asociada a un robot y un booleano que indica 
        si se obtuvo un frame válido (True si se asoció correctamente, False si se 
        tuvo que mover el robot para asignar una IP).

        Nota:
            isValidFrame es necesario por como cv2.CAP_DSHOW toma los frames
        """
        if robotIP == None:
            robotIP = self.setupMoveRobot(robotsIPs)
            isValidFrame = False
        else:
            robotIP = self.setupRobotIP(robotsIPs)
            isValidFrame = True
        
        if len(robotsIPs) == 0:
            self.printRobots()
            time.sleep(0.5)

        return robotIP, isValidFrame


    def printRobots(self):
        """
        Imprime la lista de robots encontrados y envía una instrucción para que 
        cada robot gire 90 grados en sentido antihorario.

        La función itera sobre todos los robots detectados y para cada uno, 
        envía una instrucción de giro y muestra su identificador, nombre y dirección IP.

        Returns:
        - None
        """
        print('Robots encontrados: ')
        for robot in self.robots.values():
            self.sendInstruction(robot.IP, ['TURN|-90'], False)
            print(f"\t{robot.id}. {robot.name}, con IP: {robot.IP}")


    def setupRobotIP(self, robotsIPs):
        """
        Asocia una dirección IP a un robot en función de su desplazamiento angular.

        La función itera sobre los robots detectados y verifica el desplazamiento 
        angular de cada uno. Si el desplazamiento angular es mayor o igual a 45 grados, 
        se asigna la última dirección IP disponible de la lista `robotsIPs` a ese robot 
        y se realiza la configuración de la IP.

        Parameters:
        - robotsIPs (list): Lista de direcciones IP disponibles para los robots.

        Returns:
        - None
        """
        for robot in self.robots.values():
            displacement = robot.getDisplacement()
            if displacement != None:
                _, angularDisplacement = displacement
                if abs(angularDisplacement) >= 45:
                    robotIP = robotsIPs.pop()
                    robot.setupIP(robotIP)
                    break

        return None


    def setupMoveRobot(self, robotsIPs):
        """
       Le envía instrucciones para girar al ultimo robot de la lista robotsIPs.

        La función utiliza la última dirección IP de la lista `robotsIPs` y envía 
        instrucciones al robot para que gire 90 grados y envíe un mensaje a la base. 
        Luego, espera una respuesta del robot y verifica si está listo.

        Parameters:
        - robotsIPs (list): Lista de direcciones IP disponibles para los robots.

        Returns:
        - str: La dirección IP asignada al robot.
        """
        robotIP = robotsIPs[-1]
        instructions = ['TURN|90', 'MESSAGE_BASE|1']
        self.sendInstruction(robotIP, instructions, False)

        self.sock.settimeout(2.0)
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                if addr[0] != self.baseIP and robotIP == addr[0]:
                    message = data.decode()
                    print(f"Respuesta de {addr[0]}: {message}")

                    if message == 'READY':
                        break
            except socket.timeout:
                break
        
        return robotIP


    def cameraProcessing(self):
        """
        Procesa los fotogramas de la cámara para el seguimiento y control de los robots.

        Esta función se encarga de leer los fotogramas de la cámara, 
        aplicar el procesamiento de color y buscar los robots en 
        función de su configuración. Controla el intervalo de 
        procesamiento para asegurar que los fotogramas se manejen 
        adecuadamente y gestiona la inicialización de la grabación de 
        video y el registro de tiempos.

        Durante la ejecución, busca robots en el color definido y 
        envía las posiciones de los robots encontrados.

        Retorna:
        - None
        """
        robotIP = None
        isValidFrame = True
        robotsIPs = []
        foundRobots = set()
        lastProcessedTime = time.time()
        executed = False

        while True:
            _, frame = self.camera.read()

            if time.time() - lastProcessedTime < self.processInterval or not isValidFrame:
                isValidFrame = True
                if (self.debug == 'off' or not self.threadInputAlive or not self.videoProcess.is_alive()) and executed == True:
                    self.cleanup()
                    break
                continue

            lastProcessedTime = time.time()
            frame, frameHsv = self.processFrameColors(frame)

            if len(foundRobots) < self.numRobots:
                foundRobots, foundColors = self.searchRobotsColor(self.robotsConfig)
                if len(foundRobots) == self.numRobots:
                    robotsIPs = self.searchRobotsUdp()
                    self.processFoundRobots(foundRobots, foundColors)
            elif len(robotsIPs) != 0:
                robotIP, isValidFrame = self.setupRobots(robotIP, robotsIPs)
            else:
                if not executed:
                    executed, resultsFrame = self.initializeVideoAndLogging(frameHsv.shape[:2])
                    # self.createTimeLog()

                timeLog = round((time.time() - self.startTime), 1)
                self.sendPositions(resultsFrame, timeLog)
                self.addFrame(frame, resultsFrame, timeLog)

                # processingTime = round((time.time() - lastProcessedTime) * 1000, 1)
                # self.addTimeLog(timeLog, processingTime)


    def createTimeLog(self):
        """
        Crea un registro de tiempo para el procesamiento de los robots.

        Esta función genera un archivo CSV que registra el tiempo y el 
        tiempo de procesamiento durante la ejecución del sistema. El nombre 
        del archivo se basa en la fecha y la hora actuales, así como en 
        el número de robots en operación. 

        - Se crea un directorio llamado 'Logs' (si no existe) para almacenar 
        el archivo de registro.
        - Se establece un encabezado en el archivo CSV que incluye 
        las columnas 'time' y 'processingTime'.

        Returns:
        - None
        """
        currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
        logName = f'Time_Log_{currentTime}_Robots_{self.numRobots}.csv'
        self.pathTimeLogs = os.path.join('Logs', logName)
        header = ['time' , 'processingTime']
        with open(self.pathTimeLogs, 'w', newline='') as f:
            csv.writer(f).writerow(header)


    def addTimeLog(self, timeLog, processingTime):
        """
        Agrega una entrada al registro de tiempo de procesamiento.

        Esta función añade una nueva fila al archivo de registro de tiempo 
        previamente creado, con la información del tiempo actual y el tiempo 
        de procesamiento correspondiente. La fila se agrega al final del 
        archivo CSV especificado en `self.pathTimeLogs`.

        Parámetros:
        - timeLog (str): El tiempo actual en el formato especificado 
                        para el registro.
        - processingTime (float): El tiempo de procesamiento en milisegundos 
                                que se desea registrar.

        Returns:
        - None
        """
        row = [timeLog, processingTime]
        with open(self.pathTimeLogs, 'a', newline='') as f:
            csv.writer(f).writerow(row)


    def sendPositions(self, resultsFrame, timeLog):
        """
        Envía las posiciones de los robots y actualiza el registro de posiciones.

        Esta función itera sobre cada robot registrado, obtiene su desplazamiento
        y si está disponible, envía su posición actual. Además, dibuja un círculo
        en el frame de resultados en la posición del robot y agrega la información
        de la posición al registro de posiciones.

        Parámetros:
        - resultsFrame (numpy.ndarray): El frame de resultados donde se dibujarán
                                        los círculos representando las posiciones
                                        de los robots.
        - timeLog (str): El tiempo actual que se usará para registrar las posiciones.

        Returns:
        - None
        """
        for robot in self.robots.values():
            displacement = robot.getDisplacement()
            if displacement != None:
                x, y, angle = robot.previousPose
                self.createCircle(resultsFrame, x, y)

                instruction = f'POSE|{x}|{y}|{angle}'
                self.sendInstruction(robot.IP, [instruction], False)

                self.addPositionLog(timeLog, robot.id, robot.name, robot.previousPose, displacement)


    def initializeVideoAndLogging(self, resolution):
        """
        Inicializa el proceso de grabación de video y el registro de posiciones.

        Esta función crea un frame de resultados en blanco y configura el
        proceso de grabación de video utilizando los parámetros proporcionados.
        También inicializa el registro de posiciones de los robots y dibuja
        círculos en el frame de resultados en las posiciones iniciales de
        cada robot.

        Parámetros:
        - resolution (tuple): Una tupla que contiene la altura y el ancho
                            del marco de video.

        Returns:
        - tuple: Un tuple que contiene un valor booleano indicando el éxito
                de la inicialización y el marco de resultados (resultsFrame)
                creado.
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
        for robot in self.robots.values():
            x, y, _ = robot.previousPose
            self.createCircle(resultsFrame, x, y)
            self.addPositionLog(0, robot.id, robot.name, robot.previousPose, (0, 0))

        self.inputInstruction()
        self.readUdpConnection()

        return True, resultsFrame


    def createCircle(self, resultsFrame, x, y):
        """
        Dibuja un círculo en el frame de resultados en las coordenadas especificadas.

        Esta función convierte las coordenadas (x, y) de milímetros a píxeles
        utilizando la relación de escala definida por mmPixel y dibuja un círculo
        en el frame de resultados con un radio predefinido (bigCircleRadius).

        Parámetros:
        - resultsFrame (numpy.ndarray): El frame de resultados donde se dibujará el círculo.
        - x (float): La coordenada x de la posición en milímetros.
        - y (float): La coordenada y de la posición en milímetros.
        
        Retorna:
        - None
        """
        printX = int(x / self.mmPixel)
        printY = int(y / self.mmPixel)
        cv2.circle(resultsFrame, [printX, printY], self.bigCircleRadius, (255, 0, 0), -1)


    def cleanup(self):
        """
        Libera los recursos utilizados por la cámara y cierra las ventanas de OpenCV.

        Esta función se encarga de liberar la cámara, limpiar la cola de fotogramas 
        y cerrar todas las ventanas abiertas de OpenCV. Asegura que no queden 
        recursos ocupados al finalizar la captura de video.

        Retorna:
        - None
        """
        self.camera.release()
        
        if self.frameQueue != None:
            self.frameQueue.put(None)
            time.sleep(0.2)
            while not self.frameQueue.empty():
                self.frameQueue.get()
            self.frameQueue.close()

        cv2.destroyAllWindows()


    def processFrameColors(self, frame):
        """
        Procesa un fotograma de la cámara, corrige la imagen y aplica el procesamiento 
        de color en paralelo.

        Esta función realiza la corrección de la cámara en el fotograma recibido, 
        convierte el fotograma a espacio de color HSV y luego inicia hilos para 
        procesar los colores definidos en el sistema de visión. Al finalizar el 
        procesamiento, se une a todos los hilos y si el modo de depuración está 
        habilitado, muestra el fotograma procesado.

        Parámetros:
        - frame (ndarray): El fotograma de la cámara en formato BGR que se va a procesar.

        Retorna:
        - tuple (frame, frameHsv): Una tupla que contiene el fotograma corregido en formato
                                    BGR y el fotograma en formato HSV.
        """
        frame, frameHsv = self.cameraCorrection(frame)

        threads = []
        for color in self.frameColors.values():
            t = threading.Thread(target=color.processFrame, args=(frameHsv,))
            t.start()
            threads.append(t)

        for t in threads:
            t.join()
        
        if self.debug:
            self.cameraDebug(frameHsv)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.debug = 'off'

        return frame, frameHsv 


    def addFrame(self, frame, resultsFrame, timeLog):
        """
        Agrega un fotograma y el fotograma de resultados a la cola de procesamiento, 
        y superpone información de tiempo en el fotograma.

        Esta función toma el fotograma actual y el fotograma de resultados, 
        añade un texto que indica el tiempo transcurridoa. Luego, se coloca los
        fotogramas en la cola para su procesamiento posterior.

        Parámetros:
        - frame (ndarray): El fotograma actual de la cámara en formato BGR al que se
                            le va a agregar la información de tiempo.
        - resultsFrame (ndarray): El fotograma de resultados que se va a enviar a la cola.
        - timeLog (float): El tiempo transcurrido desde el inicio del procesamiento, en segundos.

        Retorna:
        - None
        """
        text = f'Time: {timeLog} s'
        position = (2, 26)
        cv2.putText(frame, text, position, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 200), 1, cv2.LINE_AA)

        self.frameQueue.put([frame, resultsFrame])
    

    def createPositionLog(self):
        """
        Crea un archivo de registro para almacenar la posición de los robots.

        Esta función genera un archivo CSV para registrar la información de 
        la posición de los robots, incluyendo el tiempo, el ID del robot, 
        el nombre del robot, sus coordenadas (x, y), el ángulo, el 
        desplazamiento lineal y el desplazamiento angular. El nombre del 
        archivo incluye la fecha y hora actuales, así como el número de 
        robots involucrados.

        Retorna:
        - None
        """
        currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
        logName = f'Position_Log_{currentTime}_Robots_{self.numRobots}.csv'
        self.pathPositionLogs = os.path.join(self.pathPositionLogs, logName)
        header = ['time' , 'idrobot', 'robot', 'x', 'y', 'angle', 'linearDisplacement', 'angularDisplacement']
        with open(self.pathPositionLogs, 'w', newline='') as f:
            csv.writer(f).writerow(header)


    def addPositionLog(self, timeLog, id, name, position, displacement):
        """
        Agrega una entrada al registro de posiciones de los robots. Los datos 
        se agregan al final del archivo CSV.

        Parámetros:
        - timeLog (float): Tiempo transcurrido desde el inicio del proceso.
        - id (int): ID del robot.
        - name (str): Nombre del robot.
        - position (tuple): Tupla que contiene las coordenadas (x, y) y el ángulo del robot.
        - displacement (tuple): Tupla que contiene el desplazamiento lineal y angular del robot.

        Retorna:
        - None
        """
        row = [timeLog, id, name, *position, *displacement]
        with open(self.pathPositionLogs, 'a', newline='') as f:
            csv.writer(f).writerow(row)


    def sendInstructionBroadcast(self, instructions):
        """
        Envía un conjunto de instrucciones a través de broadcast a los robots.

        Esta función toma una lista de instrucciones y las envía a todos los 
        robots disponibles en la red. También imprime un mensaje en la consola
        confirmando el envío de cada instrucción.

        Parámetros:
        - instructions (list): Lista de cadenas de texto que representan las 
                            instrucciones a enviar.

        Retorna:
        - None
        """
        for instruction in instructions:
            self.sock.sendto(instruction.encode(), (self.broadcastIP, self.port))
            print(f"(Broadcast) Mensaje enviado: {instruction}")


    @runOnThread
    def sendInstruction(self, ip, instructions, printing):
        """
        Envía un conjunto de instrucciones a un robot específico.

        Esta función toma una lista de instrucciones y las envía al robot 
        cuya dirección IP se proporciona. Si se especifica, se imprime un
        mensaje en la consola confirmando el envío de cada instrucción
        junto con el nombre del robot destinatario.

        Parámetros:
        - ip (str): La dirección IP del robot al que se enviarán las instrucciones.
        - instructions (list): Lista de cadenas de texto que representan las 
                            instrucciones a enviar.
        - printing (bool): Indica si se deben imprimir mensajes de confirmación 
                        en la consola.

        Retorna:
        - None
        """
        for instruction in instructions:
            self.sock.sendto(instruction.encode(), (ip, self.port))
            name = next((robot.name for robot in self.robots.values() if robot.IP == ip), ip)
            if printing:
                print(f"Mensaje enviado a {name}: {instruction}")


    def searchRobotsUdp(self):
        """
        Busca robots en la red mediante transmisión UDP.

        Esta función envía una instrucción de configuración de inicio a 
        la dirección IP de broadcast y espera recibir respuestas de los 
        robots en la red. Si no se reciben respuestas dentro de un tiempo
        de espera, se vuelve a enviar la instrucción.

        Retorna:
        - list: Una lista de direcciones IP de los robots encontrados en la 
                red.
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
                self.sendInstructionBroadcast(instruction)

        return list(robotsIPs)


    def createConcoleLog(self):
        """
        Crea un archivo de registro para la consola.

        Esta función genera un archivo CSV para registrar los mensajes 
        resibidos desde la comunicacion UDP, incluyendo información sobre el 
        tiempo, el ID del robot y el mensaje correspondiente. El nombre 
        del archivo se basa en la fecha y hora actuales, así como en el 
        número de robots.

        Retorna:
        - None
        """
        currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
        logName = f'Console_Log_{currentTime}_Robots_{self.numRobots}.csv'
        self.pathConsolelog = os.path.join(self.pathConsolelog, logName)
        header = ['time' , 'idrobot', 'robot', 'message']
        with open(self.pathConsolelog, 'w', newline='') as f:
            csv.writer(f).writerow(header)


    def addConcoleLog(self, timeLog, id, name, message):
        """
        Agrega una entrada al registro de la consola.

        Esta función añade una fila al archivo de registro de la consola 
        con la información recibida. Esto permite llevar un 
        seguimiento de los eventos y mensajes recibidos durante la 
        ejecución del sistema.

        Parámetros:
        - timeLog (str): El tiempo en el que se registra el evento.
        - id (str): El ID del robot que envía el mensaje.
        - name (str): El nombre del robot que envía el mensaje.
        - message (str): El mensaje que se va a registrar.

        Retorna:
        - None
        """
        row = [timeLog, id, name, message]
        with open(self.pathConsolelog, 'a', newline='') as f:
            csv.writer(f).writerow(row)


    @runOnThread
    def inputInstruction(self):
        """
        Maneja la entrada de instrucciones desde la consola.

        Esta función permite al usuario ingresar instrucciones para los 
        robots en tiempo real. Las instrucciones pueden dirigirse a un 
        robot específico utilizando su ID o ser enviadas como un 
        mensaje de broadcast. Si el usuario ingresa 'BREAK', la función 
        finalizará.

        El formato de entrada debe ser 'robotId.instrucción'. Si se 
        ingresa un ID de robot no válido o un formato incorrecto, se 
        mostrará un mensaje de error.

        Retorna:
        - None
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
            else:
                print(f"Robot ID '{robotId}' no encontrado.")
        
        self.threadInputAlive = False


    @runOnThread
    def readUdpConnection(self):
        """
        Escucha y procesa mensajes recibidos a través de la conexión UDP.

        Esta función establece un bucle infinito que espera recibir datos 
        de los robots conectados a través de UDP. Cuando se recibe un 
        mensaje, se decodifica y se registra en un archivo de log. Si el 
        mensaje recibido es 'CHECK_OBSTACLE', se ignora y no se 
        imprime en la consola.

        Retorna:
        - None
        """
        self.createConcoleLog()
        self.sock.settimeout(None)
        while True:
            data, addr = self.sock.recvfrom(1024)
            ip = addr[0]
            if ip != self.baseIP:
                message = data.decode()
                timeLog = round(time.time() - self.startTime, 1)
                name, id = next(((robot.name, robot.id) for robot in self.robots.values() if robot.IP == ip), ip)
                self.addConcoleLog(timeLog, id, name, message)

                if message.split('|')[0] == 'CHECK_OBSTACLE':
                    continue
                print(f"Mensaje de {name}: {message}")



base = Base()

def main():
    """
    Punto de entrada principal para el sistema de enjambre.

    Esta función se encarga de inicializar la base del sistema y 
    gestionar la configuración de los robots a partir de un archivo 
    de configuración. Solicita al usuario la cantidad de robots 
    que participarán en la prueba, lee la configuración desde un 
    archivo JSON y luego inicia el procesamiento de la cámara 
    para el control y seguimiento de los robots.
    """
    configurationFilePath = 'configSystem.json'
    base.numRobots = int(input('Cantidad de robots en la prueba: '))

    base.readConfigFile(configurationFilePath)
    base.cameraProcessing()
    

if __name__ == '__main__':
    main()