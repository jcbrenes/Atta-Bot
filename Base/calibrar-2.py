import cv2

cap = cv2.VideoCapture(2)

# Probar configuración óptima para Logitech
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_FOCUS, 0)  # Enfoque al infinito

# Verificar configuración real
print(f"Resolución: {cap.get(3)}x{cap.get(4)}")
print(f"FPS: {cap.get(5)}")
print(f"Autofocus: {cap.get(cv2.CAP_PROP_AUTOFOCUS)}")
print(f"Focus: {cap.get(cv2.CAP_PROP_FOCUS)}")

while True:
    ret, frame = cap.read()
    if ret:
        # Mostrar resolución real en pantalla
        h, w = frame.shape[:2]
        cv2.putText(frame, f'{w}x{h}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Test Logitech', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()