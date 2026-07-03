import cv2
import time

cam = cv2.VideoCapture(2, cv2.CAP_V4L2)
cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Esperar hasta que la camara entregue frames
print("Esperando camara...", end='', flush=True)
for _ in range(60):
    ret, _ = cam.read()
    if ret:
        break
    time.sleep(0.1)
else:
    print("\nERROR: camara indice 2 no entrega frames")
    exit(1)
print(" lista")

for _ in range(30): cam.read()

detector = cv2.aruco.ArucoDetector(
    cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
    cv2.aruco.DetectorParameters()
)

print("Capturando durante 15 segundos — mueve los markers frente a la camara")
print("Guarda snapshot en aruco_snapshot.jpg cada vez que detecta algo\n")

end = time.time() + 15
last_save = 0

while time.time() < end:
    ret, frame = cam.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        id_list = ids.flatten().tolist()
        print(f"DETECTADO: IDs={id_list}  rechazados={len(rejected)}")
        if time.time() - last_save > 2:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imwrite('aruco_snapshot.jpg', cv2.resize(frame, (960, 540)))
            print("  -> snapshot guardado en aruco_snapshot.jpg")
            last_save = time.time()
    else:
        print(f"sin deteccion | rechazados={len(rejected)}")

    time.sleep(0.3)

cam.release()
print("\nListo.")
