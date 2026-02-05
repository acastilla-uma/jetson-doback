#!/bin/bash

# Script de compilación para Jetson Nano Xavier
# Uso: ./build.sh [clean]

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"

echo "=========================================="
echo "  Compilación Lidar-Cámara Fusion"
echo "=========================================="
echo "Directorio del proyecto: $PROJECT_DIR"
echo "Directorio de compilación: $BUILD_DIR"
echo ""

# Opción para limpiar compilación anterior
if [ "$1" = "clean" ]; then
    echo "Eliminando compilación anterior..."
    rm -rf "$BUILD_DIR"
fi

# Crear directorio de compilación si no existe
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p "$BUILD_DIR"
fi

# Navegar al directorio de compilación
cd "$BUILD_DIR"

# Configurar con CMake (optimizado para Jetson)
echo "Configurando con CMake..."
cmake -DJETSON_BUILD=ON ..

# Compilar
echo "Compilando..."
make -j$(nproc)

# Información de finalización
echo ""
echo "=========================================="
echo "  Compilación completada exitosamente"
echo "=========================================="
echo ""
echo "Para ejecutar:"
echo "  $BUILD_DIR/lidar_camera_fusion <output_dir> [lidar_ip] [lidar_port]"
echo ""
echo "Ejemplo:"
echo "  $BUILD_DIR/lidar_camera_fusion ~/datos 192.168.1.201 2368"
echo ""
