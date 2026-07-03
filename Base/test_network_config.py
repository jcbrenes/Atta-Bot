#!/usr/bin/env python3
"""
Script de prueba para verificar la configuración de red del sistema AttaBot.
"""
import socket
import platform
import json

def test_network_config():
    # Cargar configuración
    with open('configSystem.json', 'r') as f:
        config = json.load(f)

    udp_config = config['udp_communication']
    base_ip = udp_config['base_ip']
    broadcast_ip = udp_config['broadcast_ip']
    port = udp_config['port']
    network_interface = udp_config.get('network_interface', None)

    print("=" * 60)
    print("TEST DE CONFIGURACIÓN DE RED - ATTABOT")
    print("=" * 60)
    print(f"\nConfiguración cargada:")
    print(f"  Base IP: {base_ip}")
    print(f"  Broadcast IP: {broadcast_ip}")
    print(f"  Puerto: {port}")
    print(f"  Interfaz de red: {network_interface}")

    # Crear socket
    print(f"\n[1/5] Creando socket UDP...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    print("  ✓ Socket creado exitosamente")

    # Configurar SO_REUSEPORT en Linux
    if platform.system() == 'Linux':
        print(f"\n[2/5] Configurando SO_REUSEPORT (Linux)...")
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            print("  ✓ SO_REUSEPORT habilitado")
        except AttributeError:
            print("  ⚠ SO_REUSEPORT no disponible en esta versión de Python")

        # Vincular a interfaz específica
        if network_interface:
            print(f"\n[3/5] Vinculando socket a interfaz '{network_interface}'...")
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                               network_interface.encode())
                print(f"  ✓ Socket vinculado a interfaz {network_interface}")
            except (AttributeError, OSError) as e:
                print(f"  ✗ No se pudo vincular a interfaz: {e}")
                print("  Nota: Puede requerir permisos de root (sudo)")
                return False
        else:
            print(f"\n[3/5] Sin interfaz específica configurada, saltando...")
    else:
        print(f"\n[2-3/5] No es Linux, saltando configuraciones específicas...")

    # Hacer bind
    print(f"\n[4/5] Haciendo bind en {base_ip}:{port}...")
    try:
        sock.bind((base_ip, port))
        print(f"  ✓ Bind exitoso en {base_ip}:{port}")
    except OSError as e:
        print(f"  ✗ Error al hacer bind: {e}")
        return False

    # Probar envío de broadcast
    print(f"\n[5/5] Probando envío de broadcast a {broadcast_ip}:{port}...")
    try:
        test_message = "TEST|PING"
        sock.sendto(test_message.encode(), (broadcast_ip, port))
        print(f"  ✓ Broadcast enviado exitosamente")
        print(f"  Mensaje: {test_message}")
    except OSError as e:
        print(f"  ✗ Error al enviar broadcast: {e}")
        sock.close()
        return False

    # Cerrar socket
    sock.close()

    print("\n" + "=" * 60)
    print("✓ TODAS LAS PRUEBAS PASARON EXITOSAMENTE")
    print("=" * 60)
    return True

if __name__ == '__main__':
    success = test_network_config()
    exit(0 if success else 1)
