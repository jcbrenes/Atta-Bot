#!/usr/bin/env python3
"""
Script de verificación del sistema Atta-Bot para Linux
Detecta cámaras, interfaces de red y verifica dependencias
"""

import cv2
import socket
import platform
import subprocess
import sys

def print_section(title):
    """Imprime una sección con formato"""
    print(f"\n{'='*60}")
    print(f"  {title}")
    print('='*60)

def check_cameras():
    """Detecta todas las cámaras disponibles"""
    print_section("CÁMARAS DISPONIBLES")

    system = platform.system()
    if system == 'Linux':
        backend = cv2.CAP_V4L2
        backend_name = "V4L2"
    elif system == 'Windows':
        backend = cv2.CAP_DSHOW
        backend_name = "DirectShow"
    else:
        backend = cv2.CAP_ANY
        backend_name = "Auto"

    print(f"Sistema: {system}")
    print(f"Backend: {backend_name}\n")

    cameras_found = []
    for i in range(5):
        cap = cv2.VideoCapture(i, backend)
        if cap.isOpened():
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = int(cap.get(cv2.CAP_PROP_FPS))

            cameras_found.append(i)
            print(f"✓ Índice {i}: {width}x{height} @ {fps} FPS")

            if system == 'Linux':
                print(f"  Dispositivo: /dev/video{i}")

            cap.release()

    if not cameras_found:
        print("✗ No se encontraron cámaras")
        print("\nEn Linux, verifica con: ls -l /dev/video*")
    else:
        print(f"\n✓ Total: {len(cameras_found)} cámara(s) encontrada(s)")
        print(f"  Recomendado para configSystem.json: \"camera_index\": {cameras_found[0]}")

    return cameras_found

def check_network():
    """Verifica interfaces de red y sugiere configuración"""
    print_section("INTERFACES DE RED")

    hostname = socket.gethostname()
    print(f"Hostname: {hostname}\n")

    # Obtener todas las IPs
    try:
        interfaces = socket.getaddrinfo(hostname, None)
        ips = set()
        for item in interfaces:
            ip = item[4][0]
            if ':' not in ip:  # Filtrar IPv6
                ips.add(ip)

        print("Direcciones IP detectadas:")
        for ip in sorted(ips):
            if ip != '127.0.0.1':
                # Calcular broadcast para red /24
                parts = ip.split('.')
                broadcast = f"{parts[0]}.{parts[1]}.{parts[2]}.255"

                print(f"\n✓ IP: {ip}")
                print(f"  Broadcast sugerido: {broadcast}")
                print(f"  Red: {parts[0]}.{parts[1]}.{parts[2]}.0/24")

        print("\n✓ Configuración recomendada para configSystem.json:")
        local_ip = [ip for ip in ips if ip.startswith('192.168.')][0] if any(ip.startswith('192.168.') for ip in ips) else list(ips)[0]
        parts = local_ip.split('.')
        broadcast = f"{parts[0]}.{parts[1]}.{parts[2]}.255"

        print(f'  "base_ip": "{local_ip}"')
        print(f'  "broadcast_ip": "{broadcast}"')
        print(f'  "port": 6060')

    except Exception as e:
        print(f"✗ Error al detectar interfaces: {e}")

def check_port(port=6060):
    """Verifica si el puerto está libre"""
    print_section(f"VERIFICACIÓN DE PUERTO {port}")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        if platform.system() == 'Linux':
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

        sock.bind(('0.0.0.0', port))
        sock.close()

        print(f"✓ Puerto {port} disponible")

    except OSError as e:
        print(f"✗ Puerto {port} en uso o error: {e}")
        print("\nVerifica procesos usando el puerto:")
        print(f"  sudo netstat -tulpn | grep {port}")
        print(f"  sudo lsof -i :{port}")

def check_opencv():
    """Verifica versión de OpenCV y backends disponibles"""
    print_section("OPENCV")

    print(f"Versión: {cv2.__version__}")
    print(f"Build info:")

    # Verificar backends de video
    backends = {
        'V4L/V4L2': cv2.CAP_V4L2,
        'FFMPEG': cv2.CAP_FFMPEG,
        'GSTREAMER': cv2.CAP_GSTREAMER,
    }

    print("\nBackends de video disponibles:")
    for name, backend in backends.items():
        try:
            cap = cv2.VideoCapture(0, backend)
            if cap.isOpened():
                print(f"  ✓ {name}")
                cap.release()
            else:
                print(f"  ✗ {name} (no funcional)")
        except:
            print(f"  ✗ {name} (no compilado)")

def check_permissions():
    """Verifica permisos para acceder a cámaras en Linux"""
    if platform.system() != 'Linux':
        return

    print_section("PERMISOS (LINUX)")

    import os
    import grp

    username = os.getenv('USER')

    # Verificar si el usuario está en el grupo 'video'
    try:
        video_group = grp.getgrnam('video')
        if username in video_group.gr_mem:
            print(f"✓ Usuario '{username}' está en el grupo 'video'")
        else:
            print(f"✗ Usuario '{username}' NO está en el grupo 'video'")
            print(f"\nPara agregar:")
            print(f"  sudo usermod -a -G video {username}")
            print("  Luego cierra sesión y vuelve a entrar")
    except KeyError:
        print("⚠ Grupo 'video' no existe en el sistema")

    # Verificar permisos de /dev/video*
    try:
        result = subprocess.run(['ls', '-l', '/dev/video*'],
                              capture_output=True, text=True, shell=True)
        if result.returncode == 0:
            print("\nDispositivos de video:")
            for line in result.stdout.strip().split('\n'):
                print(f"  {line}")
        else:
            print("✗ No se encontraron dispositivos /dev/video*")
    except Exception as e:
        print(f"⚠ No se pudieron verificar permisos: {e}")

def main():
    print("""
    ╔═══════════════════════════════════════════════╗
    ║   VERIFICACIÓN DE SISTEMA ATTA-BOT (LINUX)   ║
    ╚═══════════════════════════════════════════════╝
    """)

    print(f"Sistema operativo: {platform.system()} {platform.release()}")
    print(f"Python: {sys.version}")

    check_opencv()
    check_cameras()
    check_network()
    check_port()
    check_permissions()

    print_section("RESUMEN")
    print("""
Para configurar el sistema:

1. Edita Base/configSystem.json con los valores recomendados arriba
2. Asegúrate de estar en el grupo 'video' (Linux)
3. Verifica que el puerto UDP esté libre
4. Conecta la cámara y los ESP32 a la misma red WiFi

Para ejecutar:
  cd Base
  python3 AttaBot_Base.py
    """)

if __name__ == '__main__':
    main()
