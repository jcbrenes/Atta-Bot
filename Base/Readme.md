## Base Directory

Python code for computer running as swarm base.

System requierements:
- 1080p webcam placed on ceiling
- Python version ...

## Qué hay acá

En la raíz vive lo que corre durante un experimento, y nada más. Todo lo demás
está agrupado por para qué sirve.

| | |
|---|---|
| `AttaBot_Base.py` | el programa: cámara, ArUco, protocolo UDP y logging |
| `AttaBot_GUI.py` | su interfaz |
| `configSystem.json` | configuración del sistema de visión y de la red |
| `cameraMatrix.txt`, `dist.txt`, `mask.png` | calibración de la cámara |

`configSystem.json` nombra `cameraMatrix.txt` y `dist.txt` con rutas **relativas
al directorio de trabajo**, y la Base corre desde acá. Por eso esos tres archivos
se quedan en la raíz y no en una subcarpeta: moverlos rompe el arranque en el lab
salvo que se editen a la vez el JSON y `herramientas/Calibrar.py`, que es quien
los escribe.

### `herramientas/` — análisis, banco, calibración y diagnóstico

| | |
|---|---|
| `analyze_logs.py` | métricas de navegación por corrida y por campaña |
| `scan_logs.py` | escáner de anomalías sobre todos los logs |
| `recortar_video.py` | recorta los videos por el reloj de los logs |
| `check_system.py` | verifica cámara, red y dependencias |
| `ir_check.py` | infrarrojos y sensor de color: monitor, umbral, calibración |
| `turn_check.py` | mide el error de giro contra los logs de posición |
| `aruco_test.py` | detección de markers en vivo |
| `Calibrar.py` | calibración de cámara; escribe `cameraMatrix.txt` |
| `calibrar_colores.py` | umbrales de color |
| `marker_acrilico.py` | genera los SVG de corte de los markers |
| `test_network_config.py` | prueba de la configuración de red |
| `ValidacionRandomWalk.py` | validación del random walk |

Se corren desde `Base/`, no desde adentro de `herramientas/`, porque la ruta a la
calibración de la cámara es relativa al directorio de trabajo:

```sh
python herramientas/analyze_logs.py            # sesión más reciente
python herramientas/analyze_logs.py --all      # histórico
python herramientas/scan_logs.py
```

### Directorios de datos

`PositionLogs/` y `ConsoleLogs/` guardan los logs de cada corrida; `Videos/` y
`Logs/`, el video cenital y el mapa cuadro↔tiempo. **Ninguno se versiona**: los
datos de cada campaña se archivan aparte, junto al trabajo que los usa.
