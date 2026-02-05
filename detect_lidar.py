#!/usr/bin/env python3
"""
Detector automático de Lidar Velodyne
Escanea la red local buscando el Lidar en puertos 2368 (UDP) y 8080 (TCP)
"""

import socket
import subprocess
import re
import sys
import time
import argparse
from ipaddress import IPv4Network, IPv4Address
import netifaces

class LidarDetector:
    def __init__(self, timeout=0.5):
        self.timeout = timeout
        self.found_ip = None
        self.found_port = None
    
    def get_local_network(self):
        """Obtener la red local y máscara"""
        try:
            # Obtener interfaces de red
            for iface in netifaces.interfaces():
                if iface == 'lo':  # Saltar loopback
                    continue
                
                try:
                    addrs = netifaces.ifaddresses(iface)
                    if netifaces.AF_INET in addrs:
                        addr_info = addrs[netifaces.AF_INET][0]
                        ip = addr_info.get('addr')
                        netmask = addr_info.get('netmask')
                        
                        if ip and netmask and not ip.startswith('127'):
                            print(f"✓ Red encontrada: {iface}")
                            print(f"  IP local: {ip}")
                            print(f"  Máscara: {netmask}")
                            return ip, netmask
                except:
                    continue
            
            print("✗ No se encontró interfaz de red válida")
            return None, None
            
        except Exception as e:
            print(f"Error al detectar red: {e}")
            return None, None
    
    def generate_ip_range(self, local_ip, netmask):
        """Generar rango de IPs basado en máscara"""
        try:
            network = IPv4Network(f"{local_ip}/{netmask}", strict=False)
            ips = [str(ip) for ip in network.hosts()]
            return ips
        except Exception as e:
            print(f"Error al generar rango de IPs: {e}")
            return []
    
    def check_lidar_udp(self, ip, port=2368):
        """Verificar si hay un Lidar en puerto UDP"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(self.timeout)
            
            # Enviar datos de prueba
            sock.sendto(b'TEST', (ip, port))
            
            # Intentar recibir respuesta
            try:
                data, addr = sock.recvfrom(2048)
                sock.close()
                return True
            except socket.timeout:
                sock.close()
                return False
        except:
            return False
    
    def check_lidar_tcp(self, ip, port=8080):
        """Verificar si hay un Lidar en puerto TCP"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            
            result = sock.connect_ex((ip, port))
            sock.close()
            
            return result == 0
        except:
            return False
    
    def scan_network(self, ips):
        """Escanear la red buscando Lidar"""
        print(f"\n Escaneando {len(ips)} direcciones IP...")
        print("Buscando Lidar en puerto 2368 (UDP) y 8080 (TCP web)...")
        print("(Esto puede tardar un momento...)\n")
        
        for i, ip in enumerate(ips):
            # Mostrar progreso cada 10 IPs
            if (i + 1) % 10 == 0:
                print(f"  Verificadas {i + 1}/{len(ips)} IPs...")
            
            # Prueba puerto UDP 2368 (datos Lidar)
            if self.check_lidar_udp(ip, 2368):
                print(f"\n✓ ¡Lidar encontrado en {ip}:2368!")
                self.found_ip = ip
                self.found_port = 2368
                return True
            
            # Prueba puerto TCP 8080 (interfaz web)
            if self.check_lidar_tcp(ip, 8080):
                print(f"\n✓ ¡Dispositivo Velodyne encontrado en {ip}:8080 (Web UI)!")
                self.found_ip = ip
                self.found_port = 8080
                return True
        
        return False
    
    def detect(self):
        """Ejecutar detección completa"""
        print("=" * 40)
        print("  Detector de Lidar Velodyne")
        print("=" * 40 + "\n")
        
        # Obtener red local
        local_ip, netmask = self.get_local_network()
        if not local_ip:
            return False
        
        # Generar rango de IPs
        ips = self.generate_ip_range(local_ip, netmask)
        if not ips:
            print("✗ No se pudo generar rango de IPs")
            return False
        
        # Escanear red
        found = self.scan_network(ips)
        
        print("\n" + "=" * 40)
        if found:
            print("✓ LIDAR VELODYNE ENCONTRADO")
            print("=" * 40 + "\n")
            print(f"IP:    {self.found_ip}")
            print(f"Puerto: {self.found_port}")
            print(f"\nPara ejecutar el programa:\n")
            print(f"  ./lidar_camera_fusion ~/datos_fusion {self.found_ip} {self.found_port}")
            print("\n" + "=" * 40 + "\n")
            return True
        else:
            print("✗ No se encontró Lidar Velodyne")
            print("=" * 40 + "\n")
            print("Sugerencias:")
            print("  1. Verificar que el Lidar está conectado a la red")
            print("  2. Verificar que está en la misma red que la Jetson")
            print("  3. Probar manualmente con: python3 detect_lidar.py --ip 192.168.1.201")
            print("  4. Usar: sudo nmap -sV -p 2368,8080 192.168.1.0/24\n")
            return False
    
    def check_specific_ip(self, ip, port=2368):
        """Verificar una IP específica"""
        print(f"Verificando {ip}:{port}...\n")
        
        if port == 2368:
            found = self.check_lidar_udp(ip, port)
        elif port == 8080:
            found = self.check_lidar_tcp(ip, port)
        else:
            found = self.check_lidar_tcp(ip, port)
        
        print("=" * 40)
        if found:
            print("✓ ¡Lidar encontrado!")
            print("=" * 40 + "\n")
            print(f"IP:    {ip}")
            print(f"Puerto: {port}")
            print(f"\nPara ejecutar el programa:\n")
            print(f"  ./lidar_camera_fusion ~/datos_fusion {ip} {port}")
            print("\n" + "=" * 40 + "\n")
            return True
        else:
            print(f"✗ No se encontró Lidar en {ip}:{port}")
            print("=" * 40 + "\n")
            return False

def main():
    parser = argparse.ArgumentParser(
        description='Detector automático de Lidar Velodyne',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  python3 detect_lidar.py --scan
  python3 detect_lidar.py --ip 192.168.1.201
  python3 detect_lidar.py --ip 192.168.1.201 --port 2368
        """
    )
    
    parser.add_argument('--ip', type=str, help='Verificar IP específica (ej: 192.168.1.201)')
    parser.add_argument('--port', type=int, default=2368, help='Puerto a verificar (default: 2368)')
    parser.add_argument('--scan', action='store_true', help='Escanear red local (default)')
    
    args = parser.parse_args()
    
    detector = LidarDetector(timeout=0.3)
    
    if args.ip:
        # Verificar IP específica
        success = detector.check_specific_ip(args.ip, args.port)
        sys.exit(0 if success else 1)
    else:
        # Escanear red
        success = detector.detect()
        sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
