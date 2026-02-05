#include "data_synchronizer.h"
#include <pcl/io/pcd_io.h>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <filesystem>

namespace fs = std::filesystem;

DataSynchronizer::DataSynchronizer(const std::string& output_dir, int64_t time_tolerance_ms)
    : output_dir_(output_dir), time_tolerance_(time_tolerance_ms),
      total_frames_saved_(0), frame_counter_(0) {
}

bool DataSynchronizer::initialize(std::shared_ptr<LidarReader> lidar,
                                  std::shared_ptr<CameraReader> camera) {
    lidar_reader_ = lidar;
    camera_reader_ = camera;
    
    createOutputDirectories();
    
    return true;
}

bool DataSynchronizer::start() {
    if (!lidar_reader_ || !camera_reader_) {
        std::cerr << "Error: Lidar o cámara no inicializados" << std::endl;
        return false;
    }
    
    if (!lidar_reader_->start()) {
        std::cerr << "Error al iniciar Lidar" << std::endl;
        return false;
    }
    
    if (!camera_reader_->start()) {
        std::cerr << "Error al iniciar cámara" << std::endl;
        lidar_reader_->stop();
        return false;
    }
    
    std::cout << "Sincronizador de datos iniciado" << std::endl;
    return true;
}

void DataSynchronizer::stop() {
    if (lidar_reader_) lidar_reader_->stop();
    if (camera_reader_) camera_reader_->stop();
    
    std::cout << "Sincronizador de datos detenido" << std::endl;
    std::cout << "Total de frames guardados: " << total_frames_saved_ << std::endl;
}

bool DataSynchronizer::getSynchronizedData(SynchronizedData& out_data) {
    return synchronizeData(out_data);
}

bool DataSynchronizer::processAndSave() {
    SynchronizedData sync_data;
    
    if (!synchronizeData(sync_data)) {
        return false;
    }
    
    if (!saveData(sync_data)) {
        return false;
    }
    
    total_frames_saved_++;
    
    return true;
}

bool DataSynchronizer::synchronizeData(SynchronizedData& out_data) {
    // Obtener datos más recientes
    auto cloud = lidar_reader_->getLatestCloud();
    auto rgb = camera_reader_->getRGB();
    auto depth = camera_reader_->getDepth();
    
    uint64_t lidar_ts = lidar_reader_->getTimestamp();
    uint64_t camera_ts = camera_reader_->getTimestamp();
    
    // Verificar si los datos son válidos
    if (!cloud || cloud->empty() || rgb.empty() || depth.empty()) {
        return false;
    }
    
    // Verificar sincronización temporal
    int64_t time_diff = std::abs(static_cast<int64_t>(lidar_ts - camera_ts));
    
    if (time_diff > time_tolerance_) {
        std::cerr << "Advertencia: Diferencia de tiempo > " << time_tolerance_
                  << "ms (" << time_diff << "ms)" << std::endl;
        // No retornar false para permitir datos ligeramente desincronizados
    }
    
    // Preparar datos sincronizados
    out_data.point_cloud = cloud;
    out_data.rgb_image = rgb;
    out_data.depth_image = depth;
    out_data.lidar_timestamp = lidar_ts;
    out_data.camera_timestamp = camera_ts;
    out_data.frame_id = frame_counter_++;
    
    return true;
}

bool DataSynchronizer::saveData(const SynchronizedData& data) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << data.frame_id;
    std::string frame_id_str = ss.str();
    
    try {
        // Guardar nube de puntos
        std::string pcd_file = pointcloud_dir_ + "/" + frame_id_str + ".pcd";
        if (pcl::io::savePCDFileASCII(pcd_file, *data.point_cloud) < 0) {
            std::cerr << "Error al guardar archivo PCD: " << pcd_file << std::endl;
            return false;
        }
        
        // Guardar imagen RGB
        std::string rgb_file = rgb_dir_ + "/" + frame_id_str + ".png";
        if (!cv::imwrite(rgb_file, data.rgb_image)) {
            std::cerr << "Error al guardar imagen RGB: " << rgb_file << std::endl;
            return false;
        }
        
        // Guardar mapa de profundidad
        std::string depth_file = depth_dir_ + "/" + frame_id_str + ".png";
        if (!cv::imwrite(depth_file, data.depth_image)) {
            std::cerr << "Error al guardar mapa de profundidad: " << depth_file << std::endl;
            return false;
        }
        
        // Guardar metadatos
        std::ofstream meta_file(metadata_file_, std::ios::app);
        if (meta_file.is_open()) {
            meta_file << frame_id_str << ","
                     << data.lidar_timestamp << ","
                     << data.camera_timestamp << ","
                     << (data.camera_timestamp - data.lidar_timestamp) << "\n";
            meta_file.close();
        }
        
        if (data.frame_id % 10 == 0) {
            std::cout << "Frame " << data.frame_id << " guardado exitosamente" << std::endl;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error al guardar datos: " << e.what() << std::endl;
        return false;
    }
}

void DataSynchronizer::createOutputDirectories() {
    try {
        pointcloud_dir_ = output_dir_ + "/pointclouds";
        rgb_dir_ = output_dir_ + "/rgb";
        depth_dir_ = output_dir_ + "/depth";
        metadata_file_ = output_dir_ + "/metadata.csv";
        
        fs::create_directories(pointcloud_dir_);
        fs::create_directories(rgb_dir_);
        fs::create_directories(depth_dir_);
        
        // Crear archivo de metadatos con encabezados
        std::ofstream meta_file(metadata_file_);
        if (meta_file.is_open()) {
            meta_file << "frame_id,lidar_timestamp_ms,camera_timestamp_ms,time_diff_ms\n";
            meta_file.close();
        }
        
        std::cout << "Directorios de salida creados en: " << output_dir_ << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error al crear directorios: " << e.what() << std::endl;
    }
}
