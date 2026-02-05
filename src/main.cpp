#include <iostream>
#include <memory>
#include <chrono>
#include <signal.h>
#include "lidar_reader.h"
#include "camera_reader.h"
#include "data_synchronizer.h"

volatile bool keep_running = true;

void signalHandler(int signal) {
    std::cout << "\nRecibida señal de interrupción. Deteniendo..." << std::endl;
    keep_running = false;
}

void printUsage(const char* program_name) {
    std::cout << "Uso: " << program_name << " <output_directory> [lidar_ip] [lidar_port]\n"
              << "Ejemplo: " << program_name << " ./datos 192.168.1.201 2368\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }
    
    std::string output_dir = argv[1];
    std::string lidar_ip = (argc > 2) ? argv[2] : "192.168.1.201";
    int lidar_port = (argc > 3) ? std::stoi(argv[3]) : 2368;
    
    std::cout << "========================================" << std::endl;
    std::cout << "  Fusion Lidar-Cámara para Jetson Nano Xavier" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Directorio de salida: " << output_dir << std::endl;
    std::cout << "Lidar IP: " << lidar_ip << std::endl;
    std::cout << "Lidar Port: " << lidar_port << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Configurar manejador de señales
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Crear instancias
    auto lidar = std::make_shared<LidarReader>(lidar_ip, lidar_port);
    auto camera = std::make_shared<CameraReader>(0);
    auto synchronizer = std::make_unique<DataSynchronizer>(output_dir, 50);  // 50ms tolerancia
    
    // Inicializar componentes
    std::cout << "Inicializando componentes..." << std::endl;
    
    if (!lidar->initialize()) {
        std::cerr << "Error al inicializar Lidar" << std::endl;
        return 1;
    }
    
    if (!camera->initialize()) {
        std::cerr << "Error al inicializar cámara" << std::endl;
        return 1;
    }
    
    if (!synchronizer->initialize(lidar, camera)) {
        std::cerr << "Error al inicializar sincronizador" << std::endl;
        return 1;
    }
    
    // Iniciar adquisición de datos
    std::cout << "Iniciando adquisición de datos..." << std::endl;
    if (!synchronizer->start()) {
        std::cerr << "Error al iniciar sincronizador" << std::endl;
        return 1;
    }
    
    std::cout << "Sistema en ejecución. Presione Ctrl+C para salir." << std::endl;
    std::cout << "Adquiriendo datos sincronizados...\n" << std::endl;
    
    // Loop principal de adquisición
    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (keep_running) {
        // Intentar procesar y guardar datos sincronizados
        if (synchronizer->processAndSave()) {
            frame_count++;
        }
        
        // Pequeña pausa para no sobrecargar la CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Mostrar estadísticas cada 100 frames
        if (frame_count % 100 == 0 && frame_count > 0) {
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                current_time - start_time).count();
            
            if (elapsed > 0) {
                double fps = static_cast<double>(frame_count) / elapsed;
                std::cout << "Frames guardados: " << frame_count 
                         << " | FPS: " << std::fixed << std::setprecision(2) << fps << std::endl;
            }
        }
    }
    
    // Detener adquisición
    std::cout << "\nDeteniendo adquisición..." << std::endl;
    synchronizer->stop();
    
    // Mostrar resumen final
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        end_time - start_time).count();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Resumen de ejecución" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total de frames guardados: " << synchronizer->getTotalFramesSaved() << std::endl;
    std::cout << "Tiempo de ejecución: " << total_elapsed << " segundos" << std::endl;
    
    if (total_elapsed > 0) {
        double avg_fps = static_cast<double>(synchronizer->getTotalFramesSaved()) / total_elapsed;
        std::cout << "FPS promedio: " << std::fixed << std::setprecision(2) << avg_fps << std::endl;
    }
    
    std::cout << "Directorio de salida: " << output_dir << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    return 0;
}
