# Fusión Lidar-Cámara en Python

Script Python simple para leer un **Lidar Velodyne** y una **cámara RealSense** de manera sincronizada en **Jetson Nano Xavier**.

## Ventajas vs Versión C++

✅ **Más simple**: Sin necesidad de compilación  
✅ **Más rápido de modificar**: Python es dinámico  
✅ **Menos dependencias**: Solo librerías estándar  
✅ **Debugging fácil**: Mensajes claros y tracebacks

## Instalación de Dependencias

```bash
# 1. Instalar librerías Python necesarias
pip3 install numpy opencv-python pyrealsense2

# 2. Opcional: instalar netifaces para detectar IP
pip3 install netifaces
```

## Uso

### Ejecución simple (IP/puerto por defecto)

```bash
cd ~/Documents/PhDAlex/jetson-doback/python_lidar_camera
python3 lidar_camera_fusion.py ~/datos_fusion
```

### Con parámetros personalizados

```bash
python3 lidar_camera_fusion.py ~/datos_fusion --ip 192.168.1.201 --port 2368
```

### Ver ayuda

```bash
python3 lidar_camera_fusion.py --help
```

## Estructura de Salida

```
datos_fusion/
├── pointclouds/        # Nubes de puntos (.npy - NumPy format)
│   ├── 000000.npy
│   ├── 000001.npy
│   └── ...
├── rgb/               # Imágenes RGB (.png)
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
├── depth/             # Mapas de profundidad (.png)
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
└── metadata.csv       # Información de sincronización
```

## Archivo de Metadatos (metadata.csv)

```csv
frame_id,lidar_timestamp_ms,camera_timestamp_ms,time_diff_ms
000000,1738703500123,1738703500128,5
000001,1738703500173,1738703500178,5
...
```

## Procesamiento de Datos

### Leer nube de puntos (NPY)

```python
import numpy as np

points = np.load('pointclouds/000000.npy')
print(points.shape)  # (N, 3) array
```

### Leer imágenes

```python
import cv2

rgb = cv2.imread('rgb/000000.png')
depth = cv2.imread('depth/000000.png', cv2.IMREAD_ANYDEPTH)
```

### Leer metadatos

```python
import pandas as pd

metadata = pd.read_csv('metadata.csv')
print(metadata.head())
```

## Configuración

### Cambiar resolución de cámara

En `lidar_camera_fusion.py`, línea ~120:

```python
self.camera = CameraReader(width=1280, height=720, fps=30)
```

Resoluciones soportadas: 320x240, 424x240, 640x480, 1280x720

### Ajustar tolerancia de sincronización

En `lidar_camera_fusion.py`, línea ~250:

```python
self.synchronizer = DataSynchronizer(output_dir, time_tolerance_ms=100)
```

Valores recomendados:
- 30-50ms: Sincronización estricta
- 50-100ms: Sincronización moderada
- >100ms: Permisiva

## Troubleshooting

### Error: "No module named pyrealsense2"

```bash
pip3 install pyrealsense2
```

### Error: "Cannot find Lidar"

1. Verificar conexión de red:
   ```bash
   ping 192.168.1.201
   ```

2. Usar script de detección:
   ```bash
   cd ..
   python3 detect_lidar.py --scan
   ```

### Bajo rendimiento

- Reducir resolución de cámara
- Aumentar tolerancia de sincronización
- Verificar uso de CPU:
  ```bash
  top -p $(pgrep python3)
  ```

## Comparación C++ vs Python

| Aspecto | C++ | Python |
|--------|-----|--------|
| Velocidad | ⚡⚡⚡ Muy rápido | ⚡⚡ Rápido |
| Compilación | Necesaria | No necesaria |
| Facilidad | Media | Alta |
| Depuración | Difícil | Fácil |
| Modificación | Lenta | Rápida |

**Recomendación**: Usa Python para prototipado y desarrollo. Usa C++ para producción si necesitas máximo rendimiento.

## Próximas Mejoras

- [ ] Compresión automática de archivos
- [ ] Streaming en tiempo real
- [ ] Visualización 3D online
- [ ] Grabación en ROS bag format
- [ ] Filtrado de puntos en tiempo real
