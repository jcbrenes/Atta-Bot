import cv2
import numpy as np
import json
import math

class CoordinateViewer:
    def __init__(self):
        self.camera = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.roi = None
        self.mm_pixel = None
        self.frame_colors = {}
        self.big_circle_radius = None
        
    def load_config(self, config_path='configSystem.json'):
        """Carga la configuración del sistema"""
        print("Cargando configuración...")
        
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
            
            vision_config = config['vision_system']
            
            # Cargar parámetros de cámara
            self.camera_matrix = np.loadtxt(vision_config['path_cameraMatrix'])
            self.dist_coeffs = np.loadtxt(vision_config['path_distance'])
            
            # Calcular mm/pixel
            mm_pixel_str = vision_config['mmPixel']
            num, den = map(float, mm_pixel_str.split('/'))
            self.mm_pixel = num / den
            
            # Radio del círculo grande
            self.big_circle_radius = int(vision_config['circles_radius']['big_circle'] / self.mm_pixel)
            
            print(f"✓ Configuración cargada")
            print(f"✓ Factor mm/pixel: {self.mm_pixel:.4f}")
            
            # Cargar colores
            self.load_colors(config['colors'], vision_config['circles_radius'])
            
            return True
            
        except Exception as e:
            print(f"❌ Error cargando configuración: {e}")
            return False
    
    def load_colors(self, colors_config, circles_config):
        """Carga la configuración de colores para detección"""
        for color_name, color_data in colors_config.items():
            self.frame_colors[color_name] = {
                'light': tuple(color_data['light']),
                'dark': tuple(color_data['dark']),
                'name': color_name
            }
        print(f"✓ Colores cargados: {list(self.frame_colors.keys())}")
    
    def setup_camera(self, camera_id=0):
        """Configura la cámara"""
        self.camera = cv2.VideoCapture(camera_id)
        
        # Configurar resolución (ajusta según tu configuración)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        if not self.camera.isOpened():
            raise Exception("No se pudo abrir la cámara")
        
        # Calcular matriz óptima
        h, w = 1080, 1920
        self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        
        print("✓ Cámara configurada")
    
    def apply_camera_correction(self, frame):
        """Aplica corrección de distorsión"""
        x, y, w, h = self.roi
        corrected = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, 
                                 None, self.new_camera_matrix)
        return corrected[y:y+h, x:x+w]
    
    def detect_circles(self, frame_hsv, color_config):
        """Detecta círculos de un color específico"""
        # Crear máscara de color
        mask = cv2.inRange(frame_hsv, color_config['light'], color_config['dark'])
        
        # Procesamiento morfológico
        kernel = np.ones((4, 4), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.dilate(mask, kernel, iterations=1)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        circles = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filtro mínimo
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    circles.append((cx, cy, area))
        
        return circles
    
    def pixels_to_mm(self, px, py):
        """Convierte coordenadas de píxeles a milímetros"""
        return round(px * self.mm_pixel, 1), round(py * self.mm_pixel, 1)
    
    def run_viewer(self):
        """Ejecuta el visualizador en tiempo real"""
        print("\n=== VISUALIZADOR DE COORDENADAS ===")
        print("Instrucciones:")
        print("- Coloca objetos de colores en el área de trabajo")
        print("- Verás las coordenadas en tiempo real")
        print("- Presiona 'g' para mostrar/ocultar grid de medición")
        print("- Presiona 'c' para capturar una medición")
        print("- Presiona ESC para salir\n")
        
        show_grid = True
        measurement_points = []
        
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                measurement_points.append((x, y))
                if len(measurement_points) > 10:
                    measurement_points.pop(0)
        
        cv2.namedWindow('Coordenadas en Tiempo Real')
        cv2.setMouseCallback('Coordenadas en Tiempo Real', mouse_callback)
        
        while True:
            ret, frame = self.camera.read()
            if not ret:
                continue
            
            # Aplicar corrección
            frame = self.apply_camera_correction(frame)
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            h, w = frame.shape[:2]
            
            # Dibujar grid si está activado
            if show_grid:
                self.draw_grid(frame, w, h)
            
            # Detectar y mostrar círculos de cada color
            all_detections = []
            for color_name, color_config in self.frame_colors.items():
                circles = self.detect_circles(frame_hsv, color_config)
                
                for cx, cy, area in circles:
                    # Convertir a mm
                    x_mm, y_mm = self.pixels_to_mm(cx, cy)
                    
                    # Color para visualización
                    if 'rojo' in color_name.lower() or 'red' in color_name.lower():
                        color = (0, 0, 255)
                    elif 'verde' in color_name.lower() or 'green' in color_name.lower():
                        color = (0, 255, 0)
                    elif 'azul' in color_name.lower() or 'blue' in color_name.lower():
                        color = (255, 0, 0)
                    elif 'amarillo' in color_name.lower() or 'yellow' in color_name.lower():
                        color = (0, 255, 255)
                    else:
                        color = (255, 255, 255)
                    
                    # Dibujar círculo
                    cv2.circle(frame, (cx, cy), 15, color, 2)
                    cv2.circle(frame, (cx, cy), 3, color, -1)
                    
                    # Mostrar coordenadas
                    text = f'({x_mm}, {y_mm})mm'
                    cv2.putText(frame, text, (cx + 20, cy - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(frame, color_name, (cx + 20, cy + 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                    
                    all_detections.append((color_name, x_mm, y_mm))
            
            # Dibujar puntos de medición manual
            for i, point in enumerate(measurement_points):
                cv2.circle(frame, point, 5, (255, 0, 255), -1)
                x_mm, y_mm = self.pixels_to_mm(point[0], point[1])
                cv2.putText(frame, f'P{i+1}: ({x_mm},{y_mm})', 
                           (point[0] + 10, point[1] - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
            
            # Si hay 2 puntos, mostrar distancia
            if len(measurement_points) >= 2:
                p1, p2 = measurement_points[-2], measurement_points[-1]
                cv2.line(frame, p1, p2, (255, 0, 255), 2)
                
                dist_px = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                dist_mm = dist_px * self.mm_pixel
                
                mid = ((p1[0] + p2[0])//2, (p1[1] + p2[1])//2)
                cv2.putText(frame, f'{dist_mm:.1f}mm', mid, 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            
            # Información en pantalla
            info_y = 25
            cv2.putText(frame, f'Detecciones: {len(all_detections)}', (10, info_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            info_y += 25
            cv2.putText(frame, f'mm/pixel: {self.mm_pixel:.4f}', (10, info_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            info_y += 20
            cv2.putText(frame, f'Area: {w * self.mm_pixel:.0f}x{h * self.mm_pixel:.0f}mm', 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            info_y += 25
            cv2.putText(frame, "'g'=grid 'c'=limpiar ESC=salir", (10, info_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            cv2.imshow('Coordenadas en Tiempo Real', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord('g'):
                show_grid = not show_grid
            elif key == ord('c'):
                measurement_points.clear()
                print("Puntos de medición limpiados")
        
        self.cleanup()
    
    def draw_grid(self, frame, width, height):
        """Dibuja grid de medición"""
        grid_spacing_mm = 50  # Grid cada 50mm
        grid_spacing_px = int(grid_spacing_mm / self.mm_pixel)
        
        # Líneas verticales
        for x in range(0, width, grid_spacing_px):
            cv2.line(frame, (x, 0), (x, height), (100, 100, 100), 1)
            mm_pos = x * self.mm_pixel
            if x > 0:
                cv2.putText(frame, f'{mm_pos:.0f}', (x+2, 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1)
        
        # Líneas horizontales
        for y in range(0, height, grid_spacing_px):
            cv2.line(frame, (0, y), (width, y), (100, 100, 100), 1)
            mm_pos = y * self.mm_pixel
            if y > 0:
                cv2.putText(frame, f'{mm_pos:.0f}', (2, y-2), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1)
    
    def cleanup(self):
        """Libera recursos"""
        if self.camera:
            self.camera.release()
        cv2.destroyAllWindows()


def main():
    viewer = CoordinateViewer()
    
    try:
        if not viewer.load_config():
            print("Error: No se pudo cargar la configuración")
            return
        
        viewer.setup_camera(camera_id=2)  # Ajusta el ID de tu cámara
        viewer.run_viewer()
        
    except KeyboardInterrupt:
        print("\n⚠️  Interrumpido por el usuario")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        viewer.cleanup()


if __name__ == "__main__":
    main()