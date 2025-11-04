import cv2
import numpy as np
import json
from pathlib import Path
'''
Herramienta para calibrar colores usando OpenCV y trackbars. Permite ajustar los valores HSV para diferentes colores,
visualizar el resultado en tiempo real y guardar la configuración en un archivo JSON.
'''
class ColorCalibrator:
    def __init__(self, config_path='config.json'):
        self.config_path = config_path
        self.load_config()
        self.current_color = list(self.config['colors'].keys())[0]
        self.cap = None
        self.frame = None
        self.window_name = 'Color Calibrator'
        
    def load_config(self):
        with open(self.config_path, 'r', encoding='utf-8') as f:
            self.config = json.load(f)
    
    def save_config(self):
        with open(self.config_path, 'w', encoding='utf-8') as f:
            json.dump(self.config, f, indent=4, ensure_ascii=False)
        print(f"\n✓ Configuración guardada en {self.config_path}")
    
    def create_trackbars(self):
        cv2.namedWindow(self.window_name)
        
        # Trackbars para Light (H, S, V)
        cv2.createTrackbar('Light H', self.window_name, 
                          self.config['colors'][self.current_color]['light'][0], 180, self.nothing)
        cv2.createTrackbar('Light S', self.window_name, 
                          self.config['colors'][self.current_color]['light'][1], 255, self.nothing)
        cv2.createTrackbar('Light V', self.window_name, 
                          self.config['colors'][self.current_color]['light'][2], 255, self.nothing)
        
        # Trackbars para Dark (H, S, V)
        cv2.createTrackbar('Dark H', self.window_name, 
                          self.config['colors'][self.current_color]['dark'][0], 180, self.nothing)
        cv2.createTrackbar('Dark S', self.window_name, 
                          self.config['colors'][self.current_color]['dark'][1], 255, self.nothing)
        cv2.createTrackbar('Dark V', self.window_name, 
                          self.config['colors'][self.current_color]['dark'][2], 255, self.nothing)
        
        colors_list = list(self.config['colors'].keys())
        print("\n=== CONTROLES ===")
        for i, color in enumerate(colors_list, 1):
            print(f"{i} - {color.upper()}")
        print("S - Guardar configuración")
        print("C - Capturar frame actual")
        print("ESC - Salir")
        print("=================\n")
    
    def nothing(self, x):
        pass
    
    def update_trackbars(self):
        cv2.setTrackbarPos('Light H', self.window_name, 
                          self.config['colors'][self.current_color]['light'][0])
        cv2.setTrackbarPos('Light S', self.window_name, 
                          self.config['colors'][self.current_color]['light'][1])
        cv2.setTrackbarPos('Light V', self.window_name, 
                          self.config['colors'][self.current_color]['light'][2])
        cv2.setTrackbarPos('Dark H', self.window_name, 
                          self.config['colors'][self.current_color]['dark'][0])
        cv2.setTrackbarPos('Dark S', self.window_name, 
                          self.config['colors'][self.current_color]['dark'][1])
        cv2.setTrackbarPos('Dark V', self.window_name, 
                          self.config['colors'][self.current_color]['dark'][2])
    
    def get_trackbar_values(self):
        light_h = cv2.getTrackbarPos('Light H', self.window_name)
        light_s = cv2.getTrackbarPos('Light S', self.window_name)
        light_v = cv2.getTrackbarPos('Light V', self.window_name)
        dark_h = cv2.getTrackbarPos('Dark H', self.window_name)
        dark_s = cv2.getTrackbarPos('Dark S', self.window_name)
        dark_v = cv2.getTrackbarPos('Dark V', self.window_name)
        
        return [light_h, light_s, light_v], [dark_h, dark_s, dark_v]
    
    def process_frame(self, frame, light, dark):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(light)
        upper = np.array(dark)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Detectar círculos
        mask_blur = cv2.GaussianBlur(mask, (9, 9), 2)
        circles = cv2.HoughCircles(mask_blur, cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                                   param1=50, param2=15, minRadius=10, maxRadius=60)
        
        frame_with_circles = frame.copy()
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                cv2.circle(frame_with_circles, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
                cv2.circle(frame_with_circles, (circle[0], circle[1]), 2, (0, 0, 255), 3)
        
        return mask, result, frame_with_circles, circles
    
    def run(self, camera_id=0):
        self.cap = cv2.VideoCapture(camera_id)
        
        if not self.cap.isOpened():
            print("Error: No se pudo abrir la cámara")
            return
        
        # Configurar resolución
        resolution = self.config['vision_system']['camera_resolution'].split('x')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(resolution[0]))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(resolution[1]))
        
        self.create_trackbars()
        
        # Capturar un frame inicial
        ret, self.frame = self.cap.read()
        if not ret:
            print("Error: No se pudo capturar frame")
            return
        
        print(f"Ajustando color: {self.current_color.upper()}")
        
        while True:
            # Leer nuevo frame solo si no hay uno capturado
            if self.frame is None:
                ret, frame = self.cap.read()
                if not ret:
                    break
            else:
                frame = self.frame.copy()
            
            light, dark = self.get_trackbar_values()
            mask, result, frame_with_circles, circles = self.process_frame(frame, light, dark)
            
            # Crear vista combinada
            h, w = frame.shape[:2]
            scale = 0.4
            new_h, new_w = int(h * scale), int(w * scale)
            
            frame_small = cv2.resize(frame, (new_w, new_h))
            mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_small = cv2.resize(mask_color, (new_w, new_h))
            result_small = cv2.resize(result, (new_w, new_h))
            circles_small = cv2.resize(frame_with_circles, (new_w, new_h))
            
            top_row = np.hstack([frame_small, mask_small])
            bottom_row = np.hstack([result_small, circles_small])
            combined = np.vstack([top_row, bottom_row])
            
            # Añadir información
            info_text = f"Color: {self.current_color.upper()} | Light: {light} | Dark: {dark}"
            cv2.putText(combined, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            if circles is not None:
                circle_count = len(circles[0])
                cv2.putText(combined, f"Circulos detectados: {circle_count}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow(self.window_name, combined)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:  # ESC
                break
            elif key == ord('s') or key == ord('S'):
                self.config['colors'][self.current_color]['light'] = light
                self.config['colors'][self.current_color]['dark'] = dark
                self.save_config()
            elif key == ord('c') or key == ord('C'):
                # Capturar/liberar frame
                if self.frame is None:
                    ret, self.frame = self.cap.read()
                    print("Frame capturado (presiona C nuevamente para liberar)")
                else:
                    self.frame = None
                    print("Frame liberado (modo video)")
            elif key in [ord('1'), ord('2'), ord('3'), ord('4')]:
                # Guardar valores actuales antes de cambiar
                self.config['colors'][self.current_color]['light'] = light
                self.config['colors'][self.current_color]['dark'] = dark
                
                # Cambiar de color
                colors_list = list(self.config['colors'].keys())
                idx = key - ord('1')
                if idx < len(colors_list):
                    self.current_color = colors_list[idx]
                    self.update_trackbars()
                    print(f"\nAjustando color: {self.current_color.upper()}")
                    self.frame = None  # Liberar frame capturado al cambiar de color
        
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrator = ColorCalibrator('configSystem.json')
    calibrator.run(camera_id=2)  # Cambia el ID si tienes múltiples cámaras