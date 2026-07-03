# 🔧 Troubleshooting: Múltiples ESP32 No Conectan al WiFi

## Síntoma
- **1 ESP32 conecta bien**
- **Los demás fallan al conectar**
- Credenciales verificadas como correctas

---

## 🎯 Diagnóstico Paso a Paso

### **Paso 1: Verificar Salida Serial de ESP32 que Falla**

1. Descomentar `#define DebugSerial` en línea 18 del .ino
2. Compilar y subir a un ESP32 que NO conecta
3. Abrir Serial Monitor (115200 baud)
4. Observar la salida

#### **Salida Esperada (Exitosa)**:
```
Esperando 847 ms antes de iniciar WiFi...
Iniciando WiFi con hostname: AttaBot-AABBCCDD
=== Iniciando conexión WiFi ===
SSID: Atta-Bot
MAC: AA:BB:CC:DD:EE:FF
Hostname: AttaBot-AABBCCDD
WiFi Status cambió: WL_DISCONNECTED (6)
WiFi Status cambió: WL_IDLE_STATUS (0)
WiFi Status cambió: WL_CONNECTED (3)
=== ✓ WiFi CONECTADO ===
IP: 192.168.1.150
Gateway: 192.168.1.1
MAC: AA:BB:CC:DD:EE:FF
```

#### **Salida Problemática #1: WL_NO_SSID_AVAIL**
```
WiFi Status cambió: WL_NO_SSID_AVAIL (1)
⚠ Timeout WiFi (intento #1). Estado: 1
```

**Significa**: El ESP32 no encuentra la red WiFi.

**Causas posibles**:
- Red WiFi apagada/fuera de rango
- SSID mal escrito en el código
- Red en 5GHz (ESP32 solo soporta 2.4GHz)

**Solución**:
```bash
# Verificar que la red existe
nmcli dev wifi list | grep "Atta-Bot"

# Verificar frecuencia
iwlist wlan0 scan | grep -A 5 "Atta-Bot"
# Debe mostrar "Frequency: 2.4XX GHz"
```

---

#### **Salida Problemática #2: WL_CONNECT_FAILED**
```
WiFi Status cambió: WL_CONNECT_FAILED (4)
⚠ Timeout WiFi (intento #1). Estado: 4
```

**Significa**: El router rechazó la conexión.

**Causas posibles**:
1. **Límite de clientes alcanzado** en el router
2. Filtrado MAC activo
3. Contraseña incorrecta (aunque dijiste que está bien)
4. Modo de seguridad incompatible

**Solución**:
```bash
# Ver dispositivos conectados al router
sudo nmap -sn 192.168.1.0/24

# Si ves ~10-15 dispositivos, probablemente es límite de clientes
# Opciones:
#   1. Desconectar dispositivos no esenciales
#   2. Usar un AP WiFi dedicado para los robots
#   3. Cambiar router por uno que soporte más clientes
```

---

#### **Salida Problemática #3: WL_IDLE_STATUS Loop**
```
WiFi Status cambió: WL_IDLE_STATUS (0)
WiFi Status cambió: WL_DISCONNECTED (6)
WiFi Status cambió: WL_IDLE_STATUS (0)
WiFi Status cambió: WL_DISCONNECTED (6)
... (se repite)
```

**Significa**: DHCP no asigna IP.

**Causas posibles**:
1. **Colisión DHCP** (múltiples ESP32 arrancan simultáneamente)
2. Pool DHCP lleno
3. Conflicto de IP

**Solución**: El código ahora tiene delay aleatorio. Si persiste:

**Opción A**: Asignar IPs estáticas

Agregar en `setup()` después de `WiFi.begin()`:
```cpp
// Calcular IP única basada en MAC
uint32_t macLower = (uint32_t)ESP.getEfuseMac();
uint8_t ipSuffix = 100 + (macLower % 50);  // IPs entre .100-.150

IPAddress local_IP(192, 168, 1, ipSuffix);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);

if (!WiFi.config(local_IP, gateway, subnet, dns)) {
  DebugSerialPrintln("Fallo al configurar IP estática");
}
```

**Opción B**: Aumentar pool DHCP del router
- Entrar a configuración del router (192.168.1.1)
- Buscar "DHCP Pool" o "Rango DHCP"
- Aumentar de ej: 10 dispositivos → 50 dispositivos

---

### **Paso 2: Verificar Direcciones MAC**

Si sospechas MAC duplicadas:

```cpp
// En setup(), antes de WiFi.begin():
#ifdef DebugSerial
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  delay(10000);  // Pausar para leer
#endif
```

Sube esto a **todos los ESP32** y anota sus MACs.

**Si 2 o más tienen la misma MAC**: Tienes clones baratos con MACs duplicadas.

**Solución**: Cambiar MAC manualmente
```cpp
// En setup(), ANTES de WiFi.mode()
uint8_t newMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01};  // Cambiar último byte por robot
esp_wifi_set_mac(WIFI_IF_STA, newMAC);
```

---

### **Paso 3: Método de Encendido Secuencial**

Si el delay aleatorio no es suficiente:

**Método Manual**:
1. Desconectar TODOS los ESP32
2. Conectar el primero → esperar que conecte (LED apagado)
3. Conectar el segundo → esperar
4. Repetir

Si funciona así, confirma que es problema de DHCP concurrente.

**Solución Permanente**: Usar IPs estáticas (ver Opción A arriba).

---

## 🔍 Comandos Útiles para Diagnóstico

### Ver dispositivos conectados al router:
```bash
sudo nmap -sn 192.168.1.0/24 | grep "AttaBot\|ESP"
```

### Ver tráfico UDP (verificar que ESP32 envía broadcast):
```bash
sudo tcpdump -i wlan0 udp port 6060 -n
```

### Ver logs del router (si tienes acceso SSH):
```bash
# Varía según router, ejemplo OpenWRT:
logread | grep -i dhcp
```

### Escanear WiFi con ESP32:
Agregar temporalmente en `setup()`:
```cpp
WiFi.mode(WIFI_STA);
int n = WiFi.scanNetworks();
for (int i = 0; i < n; i++) {
  Serial.printf("%d: %s (%d dBm) Ch:%d\n",
                i, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
}
```

---

## 🎛️ Configuración del Router Recomendada

Para enjambre de múltiples ESP32:

### DHCP:
- **Pool size**: Mínimo 50 IPs
- **Lease time**: 24 horas (evita renovaciones frecuentes)
- **Reservar IPs**: Opcional, mapear MAC → IP fija

### WiFi:
- **Canal fijo**: 1, 6 o 11 (evitar auto)
- **Ancho de banda**: 20MHz (más estable que 40MHz)
- **Potencia TX**: Máxima
- **Beacon interval**: 100ms (default)
- **RTS threshold**: 2347 (deshabilitado)

### Seguridad:
- **Modo**: WPA2-PSK (no WPA3, no compatible con ESP32 antiguo)
- **Cifrado**: AES
- **Filtrado MAC**: Deshabilitado (para testing)

---

## 🚨 Soluciones Rápidas por Síntoma

| Síntoma en Serial | Causa Probable | Fix Rápido |
|-------------------|----------------|------------|
| `WL_NO_SSID_AVAIL` | Red no visible | Verificar SSID y frecuencia 2.4GHz |
| `WL_CONNECT_FAILED` | Router rechaza | Ver límite de clientes, deshabilitar filtrado MAC |
| `WL_IDLE_STATUS` loop | DHCP collision | Usar IPs estáticas |
| Timeout tras 30s | Contraseña mal | Re-verificar password (case-sensitive) |
| Conecta 1 de cada 5 veces | Timing issue | Ya arreglado con delay aleatorio |

---

## ✅ Checklist de Verificación

Antes de reportar error más complejo:

- [ ] DebugSerial habilitado y revisado
- [ ] Serial Monitor muestra logs completos
- [ ] SSID y password verificados (copiar/pegar)
- [ ] Red WiFi en 2.4GHz confirmada
- [ ] Router muestra menos de 15 dispositivos conectados
- [ ] Delay aleatorio implementado (código nuevo)
- [ ] Al menos 1 ESP32 conecta exitosamente
- [ ] MACs de todos los ESP32 son **diferentes**

Si todos estos checks pasan y aún falla, el problema está en:
1. Router barato con stack WiFi buggy → Cambiar router
2. Interferencia WiFi extrema → Cambiar canal
3. Hardware defectuoso en ESP32 → Reemplazar

---

## 💡 Tip: Testing Individual

Para confirmar que un ESP32 funciona:

1. Desconectar todos los demás
2. Subir código al ESP32 problemático
3. Encender solo ese
4. Si conecta solo → Confirma que es problema de concurrencia
5. Si NO conecta solo → Hardware/código del ESP32

---

## 📞 Información para Reportar Problema

Si nada funciona, envía:

```bash
# Desde el ESP32 (Serial Monitor):
=== Iniciando conexión WiFi ===
SSID: Atta-Bot
MAC: XX:XX:XX:XX:XX:XX
WiFi Status cambió: [COPIAR AQUÍ]

# Desde Linux:
ip addr show
nmcli dev wifi list
sudo nmap -sn 192.168.1.0/24
```

---

**Última actualización**: 2025-12-02
