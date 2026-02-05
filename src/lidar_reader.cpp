#include "lidar_reader.h"
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <iostream>

LidarReader::LidarReader(std::string ip, int port)
    : lidar_ip_(ip), lidar_port_(port), latest_timestamp_(0),
      is_running_(false), has_new_data_(false) {
    latest_cloud_ = std::make_shared<PointCloud>();
}

LidarReader::~LidarReader() {
    stop();
}

bool LidarReader::initialize() {
    std::cout << "Inicializando Lidar Velodyne en " << lidar_ip_ 
              << ":" << lidar_port_ << std::endl;
    
    // La inicialización específica dependerá de la biblioteca Velodyne
    // Esta es una implementación base que debe adaptarse
    return true;
}

bool LidarReader::start() {
    if (is_running_) return false;
    
    is_running_ = true;
    read_thread_ = std::make_unique<std::thread>(&LidarReader::readThread, this);
    
    std::cout << "Lidar reader iniciado" << std::endl;
    return true;
}

void LidarReader::stop() {
    is_running_ = false;
    
    if (read_thread_ && read_thread_->joinable()) {
        read_thread_->join();
    }
    
    std::cout << "Lidar reader detenido" << std::endl;
}

PointCloud::Ptr LidarReader::getLatestCloud() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    has_new_data_ = false;
    return latest_cloud_;
}

bool LidarReader::hasNewData() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return has_new_data_;
}

uint64_t LidarReader::getTimestamp() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return latest_timestamp_;
}

void LidarReader::readThread() {
    while (is_running_) {
        try {
            // Crear una nueva nube de puntos
            auto cloud = std::make_shared<PointCloud>();
            
            // Aquí iría el código para leer del Lidar
            // Ejemplo de lectura (debe adaptarse según la SDK)
            // - Conectar al socket UDP del Lidar
            // - Procesar paquetes Velodyne
            // - Convertir a PointCloud PCL
            
            // Simulación para propósito de ejemplo:
            // En producción, esto leería datos reales del Lidar
            
            auto now = std::chrono::high_resolution_clock::now();
            uint64_t timestamp = 
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()).count();
            
            {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                latest_cloud_ = cloud;
                latest_timestamp_ = timestamp;
                has_new_data_ = true;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
        } catch (const std::exception& e) {
            std::cerr << "Error en lectura de Lidar: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}
