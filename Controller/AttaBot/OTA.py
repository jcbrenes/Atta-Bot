import subprocess
import threading
import os

# Configuración
BIN = "/home/thrain/Documents/Atta-Bot-P_ed/Controller/AttaBot/build/esp32.esp32.esp32/AttaBot.ino.bin"
PORT = 3232            # puerto OTA (ArduinoOTA usa 3232 por defecto)
PASS = "attabot1234"              # password si usaste ArduinoOTA.setPassword()

# Lista de IPs de tus ESP32
IPS = [
    "192.168.1.101",
    "192.168.1.102",
    "192.168.1.103",
    "192.168.1.104",
    "192.168.1.105",
    "192.168.1.106",
    "192.168.1.107",
    "192.168.1.108",
]

# Ruta a espota.py (ajústala si tu instalación es diferente)
ESPOTA = os.path.expanduser("/home/thrain/.arduino15/packages/esp32/hardware/esp32/3.3.7/tools/espota.py")

def upload(ip):
    print(f"🚀 Subiendo firmware a {ip} ...")
    try:
        subprocess.run(
            ["python3", ESPOTA, "-i", ip, "-p", str(PORT), "--auth", PASS, "--file", BIN],
            check=True
        )
        print(f"✅ {ip}: actualizado correctamente")
    except subprocess.CalledProcessError:
        print(f"❌ {ip}: error en la actualización")

threads = []
for ip in IPS:
    t = threading.Thread(target=upload, args=(ip,))
    t.start()
    threads.append(t)

for t in threads:
    t.join()

print("🎉 Todos los ESP32 procesados")
