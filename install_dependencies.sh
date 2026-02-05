#!/bin/bash

# Script para instalar dependencias en Jetson Nano Xavier

echo "=========================================="
echo "  Instalación de Dependencias"
echo "  Lidar-Cámara Fusion para Jetson"
echo "=========================================="
echo ""

# Verificar si es necesario actualizar
read -p "¿Desea actualizar el sistema primero? (s/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Ss]$ ]]; then
    echo "Actualizando sistema..."
    sudo apt-get update
    sudo apt-get upgrade -y
fi

echo ""
echo "Instalando dependencias generales..."
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    curl \
    wget

echo ""
echo "Instalando PCL (Point Cloud Library)..."
sudo apt-get install -y \
    libpcl-dev \
    pcl-tools

echo ""
echo "Instalando OpenCV..."
sudo apt-get install -y \
    libopencv-dev \
    python3-opencv

echo ""
echo "Instalando librealsense..."
sudo apt-get install -y \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev

# Compilar librealsense desde fuente
read -p "¿Compilar librealsense desde fuente? (s/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Ss]$ ]]; then
    TEMP_DIR="/tmp/librealsense_build"
    mkdir -p "$TEMP_DIR"
    cd "$TEMP_DIR"
    
    echo "Clonando repositorio de librealsense..."
    git clone https://github.com/IntelRealSense/librealsense.git
    
    cd librealsense
    mkdir -p build
    cd build
    
    echo "Compilando librealsense (esto puede tardar)..."
    cmake ..
    make -j$(nproc)
    
    echo "Instalando librealsense..."
    sudo make install
    
    echo "Limpiando archivos temporales..."
    cd ~
    rm -rf "$TEMP_DIR"
fi

echo ""
echo "=========================================="
echo "  Instalación completada"
echo "=========================================="
echo ""
echo "Próximos pasos:"
echo "1. Conectar el Lidar Velodyne"
echo "2. Conectar la cámara RealSense"
echo "3. Navegar a: cd ~/Jetson-DOBACK/lidar_camera_fusion"
echo "4. Ejecutar: bash build.sh"
echo "5. Ejecutar: ./build/lidar_camera_fusion ~/datos"
echo ""
