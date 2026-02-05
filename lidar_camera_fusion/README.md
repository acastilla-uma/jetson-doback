# Fusión de Lidar Velodyne y Cámara de Profundidad para Jetson Nano Xavier

Este proyecto implementa un sistema de adquisición sincronizada de datos de un **Lidar Velodyne** y una **cámara de profundidad Intel RealSense** en una **NVIDIA Jetson Nano Xavier**.

## Características

- **Adquisición sincronizada**: Captura coordinada de datos del Lidar y la cámara
- **Almacenamiento organizad**o: Los datos se guardan en carpetas estructuradas
- **Metadatos**: Registro de timestamps y información de sincronización
- **Optimizado para Jetson**: Compilación eficiente para hardware embebido
- **Thread-safe**: Uso de mutexes para acceso seguro a datos compartidos

## Estructura del Proyecto

```
lidar_camera_fusion/
├── CMakeLists.txt           # Configuración de compilación
├── include/
│   ├── lidar_reader.h       # Lectura del Lidar Velodyne
│   ├── camera_reader.h      # Lectura de cámara RealSense
│   └── data_synchronizer.h  # Sincronización y almacenamiento
├── src/
│   ├── main.cpp             # Programa principal
│   ├── lidar_reader.cpp
│   ├── camera_reader.cpp
│   └── data_synchronizer.cpp
└── README.md                # Este archivo
```

## Requisitos Previos

### Hardware
- NVIDIA Jetson Nano Xavier
- Lidar Velodyne VLP-16 (o similar)
- Cámara de profundidad Intel RealSense D435/D455
- Conexión de red/USB para los periféricos

### Software

#### 1. Actualizar el sistema
```bash
sudo apt-get update
sudo apt-get upgrade -y
```

#### 2. Instalar dependencias generales
```bash
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev
```

#### 3. Instalar PCL (Point Cloud Library)
```bash
sudo apt-get install -y \
    libpcl-dev \
    pcl-tools
```

#### 4. Instalar OpenCV
```bash
sudo apt-get install -y \
    libopencv-dev \
    python3-opencv
```

#### 5. Instalar SDK de RealSense
```bash
# Clonar el repositorio
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

# Instalar dependencias
sudo apt-get install -y \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev

# Compilar e instalar
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

#### 6. SDK del Lidar Velodyne (opcional, según modelo)
Si necesita acceso a datos específicos del Lidar:
```bash
git clone https://github.com/ros-drivers/velodyne.git
cd velodyne
mkdir build
cd build
cmake ..
make
sudo make install
```

## Compilación

### En la Jetson Nano Xavier

```bash
# Navegar al directorio del proyecto
cd ~/Jetson-DOBACK/lidar_camera_fusion

# Crear directorio de compilación
mkdir build
cd build

# Configurar con CMake (compilación optimizada para Jetson)
cmake -DJETSON_BUILD=ON ..

# Compilar
make -j$(nproc)
```

## Uso

### Ejecución básica

```bash
./lidar_camera_fusion ~/datos
```

### Con parámetros personalizados

```bash
./lidar_camera_fusion ~/datos 192.168.1.201 2368
```

Parámetros:
- `output_directory`: Ruta donde se guardarán los datos (se crea si no existe)
- `lidar_ip`: Dirección IP del Lidar (default: 192.168.1.201)
- `lidar_port`: Puerto del Lidar (default: 2368)

### Ejemplo de ejecución

```bash
cd ~/Jetson-DOBACK/lidar_camera_fusion/build
./lidar_camera_fusion /home/nvidia/datos_fusion 192.168.1.201 2368
```

## Salida y Estructura de Datos

El programa crea la siguiente estructura de directorios:

```
datos_fusion/
├── pointclouds/          # Archivos PCD del Lidar
│   ├── 000000.pcd
│   ├── 000001.pcd
│   └── ...
├── rgb/                  # Imágenes RGB
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
├── depth/                # Mapas de profundidad
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
└── metadata.csv          # Información de sincronización
```

### Archivo de Metadatos (metadata.csv)

```csv
frame_id,lidar_timestamp_ms,camera_timestamp_ms,time_diff_ms
000000,1613577600000,1613577600005,5
000001,1613577600050,1613577600055,5
...
```

## Configuración Avanzada

### Ajustar tolerancia de sincronización

En `src/main.cpp`, línea del synchronizer:
```cpp
auto synchronizer = std::make_unique<DataSynchronizer>(output_dir, 50);  // 50ms tolerancia
```

Valores recomendados:
- 30-50ms: Sincronización estricta
- 50-100ms: Sincronización moderada
- >100ms: Sincronización permisiva

### Cambiar resolución de cámara

En `src/camera_reader.cpp`:
```cpp
// Cambiar 640x480 por la resolución deseada
config_.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, RS2_FORMAT_BGR8, 30);
config_.enable_stream(RS2_STREAM_DEPTH, 0, 1280, 720, RS2_FORMAT_Z16, 30);
```

## Solución de Problemas

### Error: "No device detected"
- Verificar conexión USB/red del Lidar y cámara
- Ejecutar con permisos sudo si es necesario:
  ```bash
  sudo ./lidar_camera_fusion ~/datos
  ```

### Error: "Cannot connect to Lidar"
- Verificar dirección IP y puerto del Lidar
- Comprobar conectividad de red:
  ```bash
  ping 192.168.1.201
  ```

### Bajo rendimiento (FPS bajo)
- Reducir resolución de la cámara
- Aumentar tolerancia de sincronización
- Verificar uso de CPU/memoria:
  ```bash
  jtop
  ```

## Optimizaciones para Jetson Nano Xavier

- **Compilación optimizada**: Flag `-O3 -march=native`
- **Threads múltiples**: Lectura no-bloqueante de sensores
- **Mutexes lightweight**: Acceso eficiente a datos compartidos

## Procesamiento de Datos Guardados

### Leer archivos PCD en Python
```python
import open3d as o3d

pcd = o3d.io.read_point_cloud("pointclouds/000000.pcd")
print(pcd)
```

### Cargar imágenes en Python
```python
import cv2

rgb = cv2.imread("rgb/000000.png")
depth = cv2.imread("depth/000000.png", cv2.IMREAD_ANYDEPTH)
```

### Parsear metadatos
```python
import pandas as pd

metadata = pd.read_csv("metadata.csv")
print(metadata.head())
```

## Notas Importantes

1. **Dirección IP del Lidar**: Verificar la configuración de red del Lidar antes de ejecutar
2. **Permisos USB**: Posiblemente sea necesario ejecutar con `sudo` para acceso a dispositivos USB
3. **Espacio en disco**: Los datos PCL y imágenes requieren bastante espacio; considerar usar almacenamiento externo
4. **Sincronización**: La tolerancia temporal debe ajustarse según los requisitos de la aplicación

## Mejoras Futuras

- [ ] Compresión de archivos PCD
- [ ] Filtrado en tiempo real de nube de puntos
- [ ] Calibración de cámara automática
- [ ] Grabación en formato ROS bag
- [ ] Visualización en tiempo real

## Licencia

MIT License

## Autor

Creado para NVIDIA Jetson Nano Xavier

## Contacto y Soporte

Para problemas o sugerencias, revisar los logs de ejecución y verificar la configuración del hardware.
