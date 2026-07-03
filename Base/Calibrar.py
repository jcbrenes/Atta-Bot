import cv2
import numpy as np
import json
import time
import math

'''
Herramienta para calibrar la cámara y calcular el factor mm/píxel usando OpenCV. Permite calibrar los parámetros intrínsecos de la cámara, 
'''
class CameraCalibrator:
    def __init__(self):
        self.camera = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.roi = None
        self.mm_pixel = None
        
    def setup_camera(self, camera_id=2):
        """Inicializa la cámara con la misma configuración del código original"""
        self.camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
        
        # Usar la misma resolución que el sistema original
        width, height = 1920, 1080  # Ajusta según configuración
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        if not self.camera.isOpened():
            raise Exception("Error: No se pudo abrir la cámara")
            
        print(f"Cámara inicializada: {width}x{height}")

    def calibrate_intrinsics(self):
        """Calibra los parámetros intrínsecos de la cámara"""
        print("\n=== CALIBRACIÓN DE PARÁMETROS INTRÍNSECOS ===")
        print("Necesitas un tablero de ajedrez para la calibración")
        print("Tu tablero: 9x6 esquinas internas, cuadrados de 39mm")
        
        # Configuración del patrón - ajustado para tablero específico
        pattern_size = (9, 6)  # Esquinas internas detectables en tu tablero (largo x ancho)
        square_size = 39.0  # mm - tamaño real de tus cuadrados
        
        # Preparar puntos de referencia 3D
        pattern_points = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        pattern_points[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        pattern_points *= square_size
        
        # Arrays para almacenar puntos
        object_points = []
        image_points = []
        
        print(f"\nInstrucciones:")
        print(f"1. Mueve el tablero a diferentes posiciones y ángulos")
        print(f"2. Presiona ESPACIO cuando veas las esquinas detectadas (líneas verdes)")
        print(f"3. Captura al menos 15-20 imágenes buenas")
        print(f"4. Presiona ESC para terminar y procesar")
        print(f"5. Incluye esquinas, centro y bordes del campo de visión\n")
        
        required_images = 15
        
        while len(image_points) < required_images:
            ret, frame = self.camera.read()
            if not ret:
                continue
                
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Encontrar esquinas del tablero - optimizado para contraste bajo
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, 
                                                    cv2.CALIB_CB_ADAPTIVE_THRESH + 
                                                    cv2.CALIB_CB_NORMALIZE_IMAGE +
                                                    cv2.CALIB_CB_FILTER_QUADS)
            
            # Mostrar estado
            status_color = (0, 255, 0) if ret else (0, 0, 255)
            status_text = "DETECTADO - Presiona ESPACIO" if ret else "Mueve el tablero"
            
            cv2.putText(frame, f'Imagenes: {len(image_points)}/{required_images}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(frame, status_text, (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            
            if ret:
                # Refinar esquinas para mayor precisión
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                          criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                cv2.drawChessboardCorners(frame, pattern_size, corners, ret)
            
            cv2.imshow('Calibracion de Camara', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' ') and ret:
                object_points.append(pattern_points)
                image_points.append(corners)
                print(f"✓ Imagen {len(image_points)} capturada")
                time.sleep(0.5)  # Evitar capturas duplicadas
            elif key == 27:  # ESC
                if len(image_points) < 10:
                    print("Necesitas al menos 10 imágenes para una calibración mínima")
                    continue
                else:
                    break
        
        cv2.destroyAllWindows()
        
        print(f"\nProcesando calibración con {len(image_points)} imágenes...")
        
        # Calibrar cámara
        ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            object_points, image_points, gray.shape[::-1], None, None
        )
        
        if ret:
            # Calcular matriz óptima
            h, w = gray.shape[:2]
            self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
            )
            
            print("✓ Calibración completada exitosamente!")
            print(f"Error de reproyección RMS: {ret:.4f}")
            
            # Guardar parámetros en el formato que espera el código original
            np.savetxt('cameraMatrix.txt', self.camera_matrix)
            np.savetxt('distCoeffs.txt', self.dist_coeffs)
            
            print("✓ Archivos guardados: cameraMatrix.txt, distCoeffs.txt")
            return True
        else:
            print("❌ Error en la calibración")
            return False

    def calculate_mm_pixel_ratio(self):
        """Calcula el factor de conversión mm/píxel usando medición manual"""
        print("\n=== CÁLCULO DE FACTOR MM/PÍXEL ===")
        print("Método: Medición manual con clics")
        
        print("\nOpciones de medición:")
        print("1. Usar objeto circular (diámetro conocido)")
        print("2. Usar dos puntos separados por distancia conocida (más confiable)")
        
        method = input("Selecciona método (1 o 2): ").strip()
        
        if method == "1":
            return self.measure_with_circle()
        else:
            return self.measure_with_distance()
    
    def measure_with_distance(self):
        """Mide el factor mm/pixel usando dos puntos de distancia conocida"""
        print("\n=== MEDICIÓN CON DOS PUNTOS ===")
        print("Coloca dos marcadores (monedas, círculos, etc.) separados por una distancia conocida")
        
        known_distance = float(input("Ingresa la distancia entre los centros en mm: "))
        
        print(f"\nInstrucciones:")
        print(f"1. Haz clic en el centro del PRIMER marcador")
        print(f"2. Haz clic en el centro del SEGUNDO marcador")
        print(f"3. El sistema calculará automáticamente el factor")
        print(f"4. Presiona 'r' para reiniciar medición")
        print(f"5. Presiona ESC para cancelar\n")
        
        measurements = []
        points = []
        
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                points.append((x, y))
        
        cv2.namedWindow('Medicion mm/pixel')
        cv2.setMouseCallback('Medicion mm/pixel', mouse_callback)
        
        while True:
            ret, frame = self.camera.read()
            if not ret:
                continue
            
            # Aplicar corrección de cámara
            if self.camera_matrix is not None:
                frame = self.apply_camera_correction(frame)
            
            # Dibujar puntos seleccionados
            for i, point in enumerate(points):
                cv2.circle(frame, point, 5, (0, 0, 255), -1)
                cv2.putText(frame, f'P{i+1}', (point[0]+10, point[1]-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Si hay 2 puntos, dibujar línea y calcular
            if len(points) == 2:
                cv2.line(frame, points[0], points[1], (0, 255, 0), 2)
                
                distance_pixels = math.sqrt((points[1][0] - points[0][0])**2 + 
                                           (points[1][1] - points[0][1])**2)
                mm_per_pixel = known_distance / distance_pixels
                
                mid_point = ((points[0][0] + points[1][0])//2, 
                            (points[0][1] + points[1][1])//2)
                
                cv2.putText(frame, f'{distance_pixels:.1f}px = {known_distance}mm', 
                           mid_point, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f'mm/pixel: {mm_per_pixel:.4f}', 
                           (mid_point[0], mid_point[1]+30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, 'Presiona ESPACIO para confirmar', (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            else:
                cv2.putText(frame, f'Haz clic en punto {len(points)+1}', (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Dibujar líneas de referencia
            h, w = frame.shape[:2]
            cv2.line(frame, (w//2, 0), (w//2, h), (255, 255, 0), 1)
            cv2.line(frame, (0, h//2), (w, h//2), (255, 255, 0), 1)
            
            cv2.imshow('Medicion mm/pixel', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' ') and len(points) == 2:
                measurements.append(mm_per_pixel)
                print(f"✓ Medición {len(measurements)}: {mm_per_pixel:.4f} mm/pixel")
                points.clear()
                
                if len(measurements) >= 3:
                    avg_mm_pixel = np.mean(measurements)
                    std_mm_pixel = np.std(measurements)
                    print(f"\nPromedio de {len(measurements)} mediciones:")
                    print(f"mm/pixel: {avg_mm_pixel:.4f} ± {std_mm_pixel:.4f}")
                    
                    confirm = input("¿Aceptar este valor? (s/n): ").strip().lower()
                    if confirm == 's':
                        self.mm_pixel = avg_mm_pixel
                        print("✓ Factor mm/pixel calculado exitosamente!")
                        break
                    else:
                        measurements.clear()
                        print("Reiniciando mediciones...")
                        
            elif key == ord('r'):  # Reiniciar
                points.clear()
                print("Medición reiniciada")
            elif key == 27:  # ESC
                break
        
        cv2.destroyAllWindows()
        return self.mm_pixel is not None
    
    def measure_with_circle(self):
        """Mide usando HoughCircles (puede ser menos confiable)"""
        print("\n=== MEDICIÓN CON CÍRCULO ===")
        known_diameter = float(input("Ingresa el diámetro real del círculo en mm: "))
        
        print(f"\nSi no detecta el círculo automáticamente,")
        print(f"presiona 'm' para medición manual haciendo clic en los bordes")
        
        measurements = []
        manual_points = []
        
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                manual_points.append((x, y))
        
        cv2.namedWindow('Medicion mm/pixel')
        cv2.setMouseCallback('Medicion mm/pixel', mouse_callback)
        
        while True:
            ret, frame = self.camera.read()
            if not ret:
                continue
            
            if self.camera_matrix is not None:
                frame = self.apply_camera_correction(frame)
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Intentar detección automática
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                                      param1=50, param2=30, minRadius=10, maxRadius=300)
            
            detected = False
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                h, w = frame.shape[:2]
                center_frame = (w//2, h//2)
                
                for (x, y, r) in circles:
                    distance_to_center = math.sqrt((x - center_frame[0])**2 + (y - center_frame[1])**2)
                    if distance_to_center < 200:  # Círculo cercano al centro
                        diameter_pixels = 2 * r
                        mm_per_pixel = known_diameter / diameter_pixels
                        
                        cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                        cv2.circle(frame, (x, y), 2, (0, 255, 0), 3)
                        cv2.putText(frame, f'{diameter_pixels}px', (x-40, y-20), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv2.putText(frame, f'{mm_per_pixel:.4f}', (x-40, y+5), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv2.putText(frame, 'ESPACIO=confirmar', (10, 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        detected = True
                        break
            
            if not detected:
                cv2.putText(frame, "Presiona 'm' para medicion manual", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Mostrar puntos manuales
            for point in manual_points:
                cv2.circle(frame, point, 3, (255, 0, 0), -1)
            
            if len(manual_points) == 2:
                diameter_pixels = math.sqrt((manual_points[1][0] - manual_points[0][0])**2 + 
                                           (manual_points[1][1] - manual_points[0][1])**2)
                mm_per_pixel = known_diameter / diameter_pixels
                cv2.line(frame, manual_points[0], manual_points[1], (255, 0, 0), 2)
                cv2.putText(frame, f'{mm_per_pixel:.4f}', (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            h, w = frame.shape[:2]
            cv2.line(frame, (w//2, 0), (w//2, h), (255, 255, 0), 1)
            cv2.line(frame, (0, h//2), (w, h//2), (255, 255, 0), 1)
            
            cv2.imshow('Medicion mm/pixel', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                if detected and circles is not None:
                    measurements.append(mm_per_pixel)
                    print(f"✓ Medición {len(measurements)}: {mm_per_pixel:.4f}")
                elif len(manual_points) == 2:
                    measurements.append(mm_per_pixel)
                    print(f"✓ Medición manual {len(measurements)}: {mm_per_pixel:.4f}")
                    manual_points.clear()
                
                if len(measurements) >= 2:
                    self.mm_pixel = np.mean(measurements)
                    print(f"✓ Factor promedio: {self.mm_pixel:.4f}")
                    break
                    
            elif key == ord('m'):
                manual_points.clear()
                print("Haz clic en dos bordes opuestos del círculo")
            elif key == 27:
                break
        
        cv2.destroyAllWindows()
        return self.mm_pixel is not None

    def apply_camera_correction(self, frame):
        """Aplica la corrección de cámara igual que en el código original"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return frame
            
        x, y, w, h = self.roi
        corrected = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, 
                                 None, self.new_camera_matrix)
        return corrected[y:y+h, x:x+w]

    def test_measurements(self):
        """Permite verificar mediciones en tiempo real"""
        print("\n=== VERIFICACIÓN DE MEDICIONES ===")
        print("Coloca objetos de tamaño conocido para verificar precisión")
        print("Presiona 'c' para capturar medición, ESC para salir")
        
        while True:
            ret, frame = self.camera.read()
            if not ret:
                continue
            
            # Aplicar corrección
            if self.camera_matrix is not None:
                frame = self.apply_camera_correction(frame)
                
            # Convertir a HSV para mejor detección
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Dibujar grid de referencia cada 100mm
            self.draw_measurement_grid(frame)
            
            # Mostrar información
            cv2.putText(frame, f'Factor: {self.mm_pixel:.4f} mm/pixel', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(frame, "Presiona 'c' para medir, ESC para salir", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow('Verificacion de Mediciones', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                self.capture_measurement(frame)
            elif key == 27:  # ESC
                break
        
        cv2.destroyAllWindows()

    def draw_measurement_grid(self, frame):
        """Dibuja una grilla de medición en mm"""
        if self.mm_pixel is None:
            return
            
        h, w = frame.shape[:2]
        grid_spacing_mm = 50  # Grid cada 50mm
        grid_spacing_px = int(grid_spacing_mm / self.mm_pixel)
        
        # Líneas verticales
        for x in range(0, w, grid_spacing_px):
            cv2.line(frame, (x, 0), (x, h), (0, 255, 255), 1)
            mm_pos = x * self.mm_pixel
            cv2.putText(frame, f'{mm_pos:.0f}mm', (x+5, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # Líneas horizontales
        for y in range(0, h, grid_spacing_px):
            cv2.line(frame, (0, y), (w, y), (0, 255, 255), 1)
            mm_pos = y * self.mm_pixel
            cv2.putText(frame, f'{mm_pos:.0f}mm', (5, y-5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    def capture_measurement(self, frame):
        """Captura una medición puntual haciendo clic"""
        print("Haz clic en dos puntos para medir la distancia entre ellos")
        
        points = []
        
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                points.append((x, y))
                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
                
                if len(points) == 2:
                    # Calcular distancia
                    p1, p2 = points
                    distance_px = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                    distance_mm = distance_px * self.mm_pixel
                    
                    # Dibujar línea y distancia
                    cv2.line(frame, p1, p2, (0, 0, 255), 2)
                    mid_point = ((p1[0] + p2[0])//2, (p1[1] + p2[1])//2)
                    cv2.putText(frame, f'{distance_mm:.1f}mm', mid_point, 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    
                    print(f"Distancia medida: {distance_mm:.1f}mm")
                    cv2.imshow('Medicion', frame)
                    cv2.waitKey(2000)  # Mostrar resultado por 2 segundos
                    points.clear()
        
        cv2.setMouseCallback('Verificacion de Mediciones', mouse_callback)

    def update_config_file(self, config_path='configSystem.json'):
        """Actualiza el archivo de configuración con los nuevos parámetros"""
        if self.mm_pixel is None:
            print("❌ No hay factor mm/pixel calculado")
            return False
            
        try:
            # Leer configuración existente
            with open(config_path, 'r') as f:
                config = json.load(f)
            
            # Actualizar paths y mm/pixel
            config['vision_system']['path_cameraMatrix'] = 'cameraMatrix.txt'
            config['vision_system']['path_distance'] = 'distCoeffs.txt'
            
            # Convertir mm/pixel a fracción para mantener formato
            # Buscar una fracción simple que se aproxime
            mm_pixel_fraction = self.find_simple_fraction(self.mm_pixel)
            config['vision_system']['mmPixel'] = mm_pixel_fraction
            
            # Guardar configuración actualizada
            with open(config_path, 'w') as f:
                json.dump(config, f, indent=4)
            
            print(f"✓ Configuración actualizada en {config_path}")
            print(f"✓ mmPixel: {mm_pixel_fraction} ({self.mm_pixel:.4f})")
            return True
            
        except Exception as e:
            print(f"❌ Error actualizando configuración: {e}")
            return False

    def find_simple_fraction(self, decimal_value):
        """Encuentra una fracción simple que aproxime el valor decimal"""
        # Buscar fracciones comunes
        for denominator in range(1000, 5000, 100):
            numerator = round(decimal_value * denominator)
            if abs(numerator / denominator - decimal_value) < 0.0001:
                return f"{numerator}/{denominator}"
        
        # Si no encuentra una fracción simple, usar aproximación
        numerator = round(decimal_value * 2000)
        return f"{numerator}/2000"

    def run_full_calibration(self):
        """Ejecuta el proceso completo de calibración"""
        print("=== CALIBRACIÓN COMPLETA DEL SISTEMA ===\n")
        
        try:
            # 1. Configurar cámara
            self.setup_camera()
            
            # 2. Calibrar parámetros intrínsecos
            if not self.calibrate_intrinsics():
                print("❌ Fallo en calibración intrínseca")
                return False
            
            # 3. Calcular factor mm/pixel
            if not self.calculate_mm_pixel_ratio():
                print("❌ Fallo en cálculo de mm/pixel")
                return False
            
            # 4. Actualizar configuración
            self.update_config_file()
            
            # 5. Verificar mediciones
            print("\n¿Quieres verificar las mediciones? (y/n): ", end="")
            if input().lower().startswith('y'):
                self.test_measurements()
            
            print("\n✓ ¡Calibración completa exitosa!")
            print("\nArchivos generados:")
            print("- cameraMatrix.txt")
            print("- distCoeffs.txt") 
            print("- configSystem.json (actualizado)")
            
            return True
            
        except Exception as e:
            print(f"❌ Error durante calibración: {e}")
            return False
        finally:
            if self.camera:
                self.camera.release()
            cv2.destroyAllWindows()


def main():
    """Función principal para ejecutar la calibración"""
    calibrator = CameraCalibrator()
    
    print("Calibrador de Cámara para Sistema de Enjambre")
    print("=" * 50)
    
    while True:
        print("\nOpciones:")
        print("1. Calibración completa (recomendado)")
        print("2. Solo calibrar parámetros intrínsecos")
        print("3. Solo calcular factor mm/pixel")
        print("4. Solo verificar mediciones")
        print("5. Salir")
        
        choice = input("Selecciona una opción (1-5): ").strip()
        
        try:
            if choice == '1':
                calibrator.run_full_calibration()
            elif choice == '2':
                calibrator.setup_camera()
                calibrator.calibrate_intrinsics()
            elif choice == '3':
                calibrator.setup_camera()
                # Cargar parámetros existentes si están disponibles
                try:
                    calibrator.camera_matrix = np.loadtxt('cameraMatrix.txt')
                    calibrator.dist_coeffs = np.loadtxt('distCoeffs.txt')
                    h, w = 1080, 1920  # Ajustar según tu resolución
                    calibrator.new_camera_matrix, calibrator.roi = cv2.getOptimalNewCameraMatrix(
                        calibrator.camera_matrix, calibrator.dist_coeffs, (w, h), 1, (w, h)
                    )
                except:
                    print("⚠️  No se encontraron parámetros de calibración previos")
                calibrator.calculate_mm_pixel_ratio()
            elif choice == '4':
                calibrator.setup_camera()
                try:
                    calibrator.camera_matrix = np.loadtxt('cameraMatrix.txt')
                    calibrator.dist_coeffs = np.loadtxt('distCoeffs.txt')
                    h, w = 1080, 1920  # Ajustar según tu resolución
                    calibrator.new_camera_matrix, calibrator.roi = cv2.getOptimalNewCameraMatrix(
                        calibrator.camera_matrix, calibrator.dist_coeffs, (w, h), 1, (w, h)
                    )
                    # Cargar mm/pixel desde config si existe
                    with open('configSystem.json', 'r') as f:
                        config = json.load(f)
                        mm_pixel_str = config['vision_system']['mmPixel']
                        num, den = map(float, mm_pixel_str.split('/'))
                        calibrator.mm_pixel = num / den
                except Exception as e:
                    print(f"⚠️  Error cargando parámetros: {e}")
                    calibrator.mm_pixel = 0.0005  # Valor por defecto
                calibrator.test_measurements()
            elif choice == '5':
                break
            else:
                print("❌ Opción inválida")
                
        except KeyboardInterrupt:
            print("\n⚠️  Calibración interrumpida por el usuario")
        except Exception as e:
            print(f"❌ Error: {e}")
        finally:
            if calibrator.camera:
                calibrator.camera.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()