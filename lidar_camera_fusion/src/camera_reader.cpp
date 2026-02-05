#include "camera_reader.h"
#include <chrono>
#include <iostream>

CameraReader::CameraReader(int device_id)
    : device_id_(device_id), is_running_(false), has_new_frame_(false) {
}

CameraReader::~CameraReader() {
    stop();
}

bool CameraReader::initialize() {
    std::cout << "Inicializando cámara RealSense (dispositivo " 
              << device_id_ << ")" << std::endl;
    
    try {
        // Configurar la cámara RealSense
        config_.enable_stream(RS2_STREAM_COLOR, 0, 640, 480, RS2_FORMAT_BGR8, 30);
        config_.enable_stream(RS2_STREAM_DEPTH, 0, 640, 480, RS2_FORMAT_Z16, 30);
        
        // Iniciar el pipeline
        auto profile = pipeline_.start(config_);
        
        // Habilitar alineamiento de frames
        auto sensor = profile.get_device().first<rs2::depth_sensor>();
        if (sensor) {
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
        }
        
        std::cout << "Cámara RealSense inicializada correctamente" << std::endl;
        return true;
        
    } catch (const rs2::error& e) {
        std::cerr << "Error al inicializar cámara RealSense: " << e.what() << std::endl;
        return false;
    }
}

bool CameraReader::start() {
    if (is_running_) return false;
    
    is_running_ = true;
    capture_thread_ = std::make_unique<std::thread>(&CameraReader::captureThread, this);
    
    std::cout << "Captura de cámara iniciada" << std::endl;
    return true;
}

void CameraReader::stop() {
    is_running_ = false;
    
    if (capture_thread_ && capture_thread_->joinable()) {
        capture_thread_->join();
    }
    
    pipeline_.stop();
    std::cout << "Captura de cámara detenida" << std::endl;
}

cv::Mat CameraReader::getRGB() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    has_new_frame_ = false;
    return latest_rgb_.clone();
}

cv::Mat CameraReader::getDepth() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_depth_.clone();
}

uint64_t CameraReader::getTimestamp() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return latest_timestamp_;
}

bool CameraReader::hasNewFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return has_new_frame_;
}

void CameraReader::captureThread() {
    while (is_running_) {
        try {
            // Esperar a que lleguen los frames
            auto frames = pipeline_.wait_for_frames(1000);  // timeout 1000ms
            
            if (!frames) continue;
            
            // Alinear frames de profundidad con RGB
            rs2::align align(RS2_STREAM_COLOR);
            frames = align.process(frames);
            
            // Obtener frames de color y profundidad
            rs2::video_frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            
            if (!color_frame || !depth_frame) continue;
            
            // Convertir a OpenCV Mat
            cv::Mat rgb(cv::Size(color_frame.get_width(), color_frame.get_height()),
                       CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            
            cv::Mat depth(cv::Size(depth_frame.get_width(), depth_frame.get_height()),
                         CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
            
            auto now = std::chrono::high_resolution_clock::now();
            uint64_t timestamp = 
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()).count();
            
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_rgb_ = rgb.clone();
                latest_depth_ = depth.clone();
                latest_timestamp_ = timestamp;
                has_new_frame_ = true;
            }
            
        } catch (const rs2::error& e) {
            std::cerr << "Error en captura de cámara: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}
