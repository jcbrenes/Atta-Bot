# Guía de Configuración: Atta-Bot en Arch Linux

## 🐧 Migración desde Windows

Este documento cubre los cambios necesarios para ejecutar Atta-Bot en Arch Linux después de trabajar en Windows.

---

## ✅ Cambios Implementados

### 1. **Backend de Cámara Multiplataforma**

El sistema ahora detecta automáticamente:
- **Linux**: `CAP_V4L2` (Video4Linux2)
- **Windows**: `CAP_DSHOW` (DirectShow)
- **macOS**: `CAP_AVFOUNDATION`

**Auto-detección de cámara**: Ya no necesitas hardcodear el índice. El sistema busca automáticamente entre `/dev/video0` - `/dev/video4`.

### 2. **Configuración UDP Mejorada**

- Agregado `SO_REUSEPORT` para Linux (evita "Address already in use")
- Mensajes de error detallados con soluciones
- Validación automática de bind

### 3. **Script de Diagnóstico**

Creado `Base/check_system.py` para verificar:
- Cámaras disponibles
- Interfaces de red
- Puertos UDP
- Permisos de usuario
- Versión de OpenCV

---

## 🚀 Setup Inicial en Arch Linux

### Paso 1: Instalar Dependencias

```bash
# Paquetes del sistema
sudo pacman -S python python-pip python-opencv v4l-utils

# Dependencias Python
cd /home/thrain/Documents/Atta-Bot-P_ed/Base
pip install opencv-python numpy
```

### Paso 2: Verificar Permisos de Cámara

```bash
# Agregar tu usuario al grupo 'video'
sudo usermod -a -G video $USER

# Aplicar cambios (cierra sesión y vuelve a entrar, o ejecuta):
newgrp video

# Verificar que la cámara es accesible
ls -l /dev/video*
```

### Paso 3: Ejecutar Diagnóstico

```bash
cd Base
python3 check_system.py
```

**Ejemplo de salida esperada:**
```
✓ Índice 0: 1920x1080 @ 30 FPS
  Dispositivo: /dev/video0

✓ IP: 192.168.1.100
  Broadcast sugerido: 192.168.1.255

✓ Puerto 6060 disponible
```

### Paso 4: Actualizar `configSystem.json`

Basándote en la salida del script de diagnóstico:

```json
{
  "vision_system": {
    "camera_index": 0,  // <-- Agregar esta línea (opcional)
    "camera_resolution": "1920x1080",
    // ... resto de configuración
  },
  "udp_communication": {
    "base_ip": "192.168.1.100",      // <-- Tu IP en Linux
    "broadcast_ip": "192.168.1.255",  // <-- Broadcast de tu red
    "port": 6060
  }
}
```

**Nota**: Si omites `camera_index`, el sistema auto-detectará la primera cámara disponible.

---

## 🔧 Solución de Problemas Comunes

### ❌ Error: "No se pudo abrir la cámara"

**Solución 1**: Verificar que existe
```bash
ls /dev/video*
v4l2-ctl --list-devices
```

**Solución 2**: Verificar permisos
```bash
# Debe mostrar crw-rw----+ 1 root video
ls -l /dev/video0

# Si no tienes permisos:
sudo usermod -a -G video $USER
newgrp video
```

**Solución 3**: Probar índice diferente
```bash
python3 check_system.py  # Te dirá qué índices están disponibles
```

---

### ❌ Error: "Address already in use" (Puerto UDP)

**Causa**: El puerto 6060 está en uso o no se liberó después de Ctrl+C.

**Solución 1**: Esperar 60 segundos (timeout de `TIME_WAIT`)

**Solución 2**: Verificar qué proceso lo usa
```bash
sudo netstat -tulpn | grep 6060
# o
sudo lsof -i :6060
```

**Solución 3**: Matar proceso anterior
```bash
# Si ves PID en la salida anterior:
kill -9 <PID>
```

**Solución 4**: Cambiar puerto temporalmente en `configSystem.json`
```json
"port": 6061
```

---

### ❌ Error: "No module named 'cv2'"

```bash
# Opción 1: pip del sistema
sudo pacman -S python-opencv

# Opción 2: pip de usuario
pip install opencv-python

# Opción 3: entorno virtual (recomendado)
python -m venv venv
source venv/bin/activate
pip install opencv-python numpy
```

---

### ❌ Cámara detectada pero imagen negra

**Causa**: La cámara puede estar en uso por otra aplicación (Zoom, Teams, etc.)

```bash
# Verificar aplicaciones usando la cámara
lsof /dev/video0

# Cerrar aplicaciones y reintentar
```

---

### ❌ ESP32 no se conectan a WiFi

**Verificación en Linux**:
```bash
# Ver dispositivos conectados a tu red
sudo nmap -sn 192.168.1.0/24

# Escuchar broadcasts UDP (en otra terminal)
sudo tcpdump -i wlan0 udp port 6060
```

**Checklist**:
1. ✓ ESP32 y PC en la misma red WiFi
2. ✓ SSID y contraseña correctos en `AttaBot.ino`
3. ✓ Firewall no bloquea UDP 6060:
   ```bash
   sudo ufw allow 6060/udp
   # o
   sudo iptables -A INPUT -p udp --dport 6060 -j ACCEPT
   ```

---

## 📊 Diferencias Windows vs Linux

| Aspecto | Windows | Linux (Arch) |
|---------|---------|--------------|
| Backend cámara | `CAP_DSHOW` | `CAP_V4L2` |
| Dispositivo cámara | Índice numérico | `/dev/videoX` |
| Socket UDP | `SO_REUSEADDR` | `SO_REUSEADDR` + `SO_REUSEPORT` |
| Permisos cámara | No requeridos | Grupo `video` |
| Rutas archivos | `\` | `/` (ya soportado con `os.path.join`) |

---

## 🧪 Prueba Rápida del Sistema

### 1. Verificar Cámara
```bash
cd Base
python3 check_system.py
```

### 2. Probar Captura de Video
```bash
cd Base
python3 -c "import cv2; cap = cv2.VideoCapture(0, cv2.CAP_V4L2); print('OK' if cap.isOpened() else 'FAIL'); cap.release()"
```

### 3. Ejecutar Sistema Base
```bash
cd Base
python3 AttaBot_Base.py
```

**Salida esperada:**
```
Detectando cámara en Linux...
✓ Cámara encontrada en índice 0
Backend de cámara: 4 (índice 0)
✓ SO_REUSEPORT habilitado (Linux)
✓ Socket UDP bind exitoso en 192.168.1.100:6060
Cantidad de robots en la prueba:
```

---

## 🎯 Tips para Arch Linux

### Optimizar Latencia de Cámara

```bash
# Deshabilitar buffering de V4L2
v4l2-ctl -d /dev/video0 --set-ctrl=video_bitrate=25000000
```

### Firewall (UFW)

```bash
# Permitir tráfico local
sudo ufw allow from 192.168.1.0/24 to any port 6060 proto udp
```

### Rendimiento

Si experimentas lag en el sistema de visión:

```python
# En AttaBot_Base.py, reducir calidad de video
self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Agregar después de línea 780
```

---

## 📝 Notas Finales

### Auto-detección vs Manual

**Auto-detección** (recomendado):
- Omite `camera_index` en `configSystem.json`
- El sistema encuentra la primera cámara disponible

**Manual** (para múltiples cámaras):
```json
"camera_index": 2  // Si tienes webcam en 0, otra en 1, y quieres la de índice 2
```

### Calibración de Cámara

Los archivos de calibración (`cameraMatrix.txt`, `distance.txt`) son compatibles entre Windows y Linux. **No necesitas recalibrar**.

---

## 🆘 Ayuda Adicional

Si sigues teniendo problemas:

1. Ejecuta el diagnóstico completo:
   ```bash
   python3 Base/check_system.py > diagnostico.txt
   ```

2. Revisa los logs con más detalle ejecutando con Python en modo verbose:
   ```bash
   python3 -v Base/AttaBot_Base.py
   ```

3. Verifica versiones:
   ```bash
   python --version
   python -c "import cv2; print(cv2.__version__)"
   ```

---

**Última actualización**: 2025-12-02
**Sistema probado**: Arch Linux (kernel 6.12.59-1-lts)
