#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <thread>
#include <atomic>

struct LidarInfo {
    std::string ip;
    int port;
    bool found;
};

// Obtener la red local y máscara
std::pair<std::string, std::string> getLocalNetwork() {
    struct ifaddrs *ifaddr, *ifa;
    std::string local_ip, netmask;

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return {"", ""};
    }

    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL) continue;

        // Buscar IPv4, saltando lo (loopback)
        if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name, "lo") != 0) {
            local_ip = inet_ntoa(((struct sockaddr_in *)ifa->ifa_addr)->sin_addr);
            netmask = inet_ntoa(((struct sockaddr_in *)ifa->ifa_netmask)->sin_addr);
            std::cout << "Red encontrada: " << ifa->ifa_name 
                     << " | IP: " << local_ip 
                     << " | Máscara: " << netmask << std::endl;
            freeifaddrs(ifaddr);
            return {local_ip, netmask};
        }
    }

    freeifaddrs(ifaddr);
    return {"", ""};
}

// Generar rango de IPs basado en máscara
std::vector<std::string> generateIPRange(const std::string& local_ip, const std::string& netmask) {
    std::vector<std::string> ips;
    
    // Parsear IP local
    struct in_addr addr, mask_addr;
    inet_aton(local_ip.c_str(), &addr);
    inet_aton(netmask.c_str(), &mask_addr);
    
    uint32_t network = addr.s_addr & mask_addr.s_addr;
    uint32_t broadcast = network | ~mask_addr.s_addr;
    
    // Generar rango de IPs
    for (uint32_t ip = network + 1; ip < broadcast; ip++) {
        struct in_addr ip_addr;
        ip_addr.s_addr = ip;
        ips.push_back(inet_ntoa(ip_addr));
    }
    
    return ips;
}

// Intentar conectar a un dispositivo en un puerto específico
bool checkLidar(const std::string& ip, int port, int timeout_ms = 500) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        return false;
    }

    // Configurar timeout
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

    // Intentar enviar datos de prueba
    const char* test_data = "TEST";
    if (sendto(sock, test_data, strlen(test_data), 0, 
               (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        close(sock);
        return false;
    }

    // Intentar recibir respuesta (Lidar típicamente envía datos)
    char buffer[2048];
    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);
    
    ssize_t n = recvfrom(sock, buffer, sizeof(buffer), 0, 
                        (struct sockaddr *)&src_addr, &src_len);
    
    close(sock);
    return (n > 0);
}

// Función para verificar puerto TCP (alternativo)
bool checkLidarTCP(const std::string& ip, int port, int timeout_ms = 500) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        return false;
    }

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

    int result = connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    close(sock);
    
    return (result == 0);
}

// Buscar un Lidar en el rango de IPs
LidarInfo scanForLidar(const std::vector<std::string>& ips, int num_threads = 4) {
    LidarInfo result = {"", 2368, false};
    
    std::cout << "\nEscaneando " << ips.size() << " direcciones IP..." << std::endl;
    std::cout << "Buscando Lidar en puerto 2368 (UDP) y 8080 (TCP web)..." << std::endl;
    std::cout << "(Esto puede tardar un momento...)\n" << std::endl;

    size_t checked = 0;
    
    for (const auto& ip : ips) {
        checked++;
        
        // Mostrar progreso cada 10 IPs
        if (checked % 10 == 0) {
            std::cout << "  Verificadas " << checked << "/" << ips.size() << " IPs..." << std::endl;
        }

        // Prueba puerto UDP 2368 (datos Lidar)
        if (checkLidar(ip, 2368, 300)) {
            std::cout << "\n✓ ¡Lidar encontrado!" << std::endl;
            result.ip = ip;
            result.port = 2368;
            result.found = true;
            return result;
        }

        // Prueba puerto TCP 8080 (interfaz web)
        if (checkLidarTCP(ip, 8080, 300)) {
            std::cout << "\n✓ ¡Dispositivo Velodyne encontrado (Web UI)!" << std::endl;
            result.ip = ip;
            result.port = 8080;
            result.found = true;
            return result;
        }
    }

    return result;
}

void printUsage(const char* program_name) {
    std::cout << "Uso: " << program_name << " [opciones]\n\n"
              << "Opciones:\n"
              << "  --ip <dirección>      Verificar IP específica (ej: 192.168.1.201)\n"
              << "  --port <puerto>       Puerto a verificar (default: 2368)\n"
              << "  --scan                Escanear red local (default)\n"
              << "  --help                Mostrar esta ayuda\n\n"
              << "Ejemplos:\n"
              << "  " << program_name << " --scan\n"
              << "  " << program_name << " --ip 192.168.1.201\n"
              << "  " << program_name << " --ip 192.168.1.201 --port 2368\n";
}

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "  Detector Automático de Lidar Velodyne" << std::endl;
    std::cout << "========================================\n" << std::endl;

    std::string check_ip = "";
    int check_port = 2368;
    bool do_scan = true;

    // Procesar argumentos
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        else if (arg == "--ip" && i + 1 < argc) {
            check_ip = argv[++i];
            do_scan = false;
        }
        else if (arg == "--port" && i + 1 < argc) {
            check_port = std::stoi(argv[++i]);
        }
        else if (arg == "--scan") {
            do_scan = true;
        }
    }

    // Verificar IP específica
    if (!check_ip.empty()) {
        std::cout << "Verificando " << check_ip << ":" << check_port << "..." << std::endl;
        
        bool found = false;
        if (check_port == 2368) {
            found = checkLidar(check_ip, check_port);
        } else if (check_port == 8080) {
            found = checkLidarTCP(check_ip, check_port);
        } else {
            found = checkLidarTCP(check_ip, check_port);
        }

        if (found) {
            std::cout << "\n✓ ¡Lidar encontrado!" << std::endl;
            std::cout << "  IP: " << check_ip << std::endl;
            std::cout << "  Puerto: " << check_port << std::endl;
            std::cout << "\nUsa estos parámetros al ejecutar:" << std::endl;
            std::cout << "  ./lidar_camera_fusion ~/datos " << check_ip << " " << check_port << std::endl;
            return 0;
        } else {
            std::cout << "\n✗ No se encontró Lidar en " << check_ip << ":" << check_port << std::endl;
            return 1;
        }
    }

    // Escanear red local
    if (do_scan) {
        auto [local_ip, netmask] = getLocalNetwork();
        
        if (local_ip.empty()) {
            std::cerr << "Error: No se pudo obtener información de la red local" << std::endl;
            return 1;
        }

        auto ips = generateIPRange(local_ip, netmask);
        
        if (ips.empty()) {
            std::cerr << "Error: No se pudo generar rango de IPs" << std::endl;
            return 1;
        }

        auto result = scanForLidar(ips);

        std::cout << "\n========================================" << std::endl;
        if (result.found) {
            std::cout << "✓ LIDAR VELODYNE ENCONTRADO" << std::endl;
            std::cout << "========================================\n" << std::endl;
            std::cout << "IP:    " << result.ip << std::endl;
            std::cout << "Puerto: " << result.port << std::endl;
            std::cout << "\nPara ejecutar el programa:\n" << std::endl;
            std::cout << "  ./lidar_camera_fusion ~/datos_fusion " << result.ip << " " << result.port << std::endl;
            std::cout << "\n========================================\n" << std::endl;
            return 0;
        } else {
            std::cout << "✗ No se encontró Lidar Velodyne" << std::endl;
            std::cout << "========================================\n" << std::endl;
            std::cout << "Sugerencias:\n"
                     << "  1. Verificar que el Lidar está conectado a la red\n"
                     << "  2. Verificar que está en la misma red que la Jetson\n"
                     << "  3. Probar manualmente con: detect_lidar --ip 192.168.1.201\n"
                     << "  4. Usar: sudo nmap -sV -p 2368,8080 192.168.1.0/24\n"
                     << std::endl;
            return 1;
        }
    }

    return 0;
}
