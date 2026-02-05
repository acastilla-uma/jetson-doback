#!/usr/bin/env python3
"""
Fusión sincronizada de Lidar Velodyne y Cámara de Profundidad RealSense
Para NVIDIA Jetson Nano Xavier
"""

import socket
import threading
import cv2
import numpy as np
import os
import sys
import time
import argparse
import struct
from datetime import datetime
from pathlib import Path
from collections import deque
from queue import Queue
import json

try:
    import pyrealsense2 as rs
except ImportError:
    print("Error: pyrealsense2 no está instalado")
    print("Instalar con: pip3 install pyrealsense2")
    sys.exit(1)


class LidarReader:
    """Lee datos del Lidar Velodyne por UDP"""
    
    def __init__(self, lidar_ip='192.168.1.201', lidar_port=2368, buffer_size=3):
        self.lidar_ip = lidar_ip
        self.lidar_port = lidar_port
        self.buffer_size = buffer_size
        self.running = False
        self.thread = None
        self.points = None
        self.timestamp = 0
        self.lock = threading.Lock()
        self.has_new_data = False
        
    def start(self):
        """Iniciar lectura del Lidar"""
        self.running = True
        self.thread = threading.Thread(target=self._read_thread, daemon=True)
        self.thread.start()
        print(f"✓ Lidar iniciado en {self.lidar_ip}:{self.lidar_port}")
    
    def stop(self):
        """Detener lectura del Lidar"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        print("✓ Lidar detenido")
    
    def _read_thread(self):
        """Thread para lectura de datos UDP"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', self.lidar_port))
            sock.settimeout(1.0)
            
            frame_count = 0
            while self.running:
                try:
                    data, addr = sock.recvfrom(1248)
                    
                    if len(data) < 1200:
                        continue
                    
                    frame_count += 1
                    timestamp_ms = int(time.time() * 1000)
                    
                    # Procesar datos del Lidar (simplificado)
                    points = self._parse_velodyne_packet(data)
                    
                    with self.lock:
                        self.points = points
                        self.timestamp = timestamp_ms
                        self.has_new_data = True
                    
                    if frame_count % 100 == 0:
                        print(f"  Lidar: {frame_count} paquetes recibidos")
                        
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"Error en lectura Lidar: {e}")
                    time.sleep(0.1)
        
        except Exception as e:
            print(f"Error al iniciar Lidar: {e}")
    
    def _parse_velodyne_packet(self, data):
        """Parsear paquete Velodyne UDP"""
        try:
            # Estructura simplificada de paquete Velodyne
            # Formato: cada canal tiene 32 bytes con datos de rango/intensidad
            points = []
            
            for i in range(0, min(len(data)-10, 1200), 6):
                # Lectura simplificada de datos
                distance = struct.unpack('<H', data[i:i+2])[0] * 0.002  # 2mm por unidad
                intensity = data[i+2] if i+2 < len(data) else 0
                
                if distance > 0.1:  # Filtrar distancias inválidas
                    points.append([0, 0, distance])  # x, y, z simplificados
            
            return np.array(points) if points else np.zeros((0, 3))
        except:
            return np.zeros((0, 3))
    
    def get_data(self):
        """Obtener últimos datos del Lidar"""
        with self.lock:
            has_new = self.has_new_data
            self.has_new_data = False
            return self.points, self.timestamp, has_new


class CameraReader:
    """Lee cámara de profundidad RealSense"""
    
    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.running = False
        self.thread = None
        self.rgb = None
        self.depth = None
        self.timestamp = 0
        self.lock = threading.Lock()
        self.has_new_frame = False
        self.pipeline = None
        self.align = None
        
    def start(self):
        """Iniciar lectura de cámara"""
        try:
            # Configurar pipeline RealSense
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            
            # Iniciar pipeline
            profile = self.pipeline.start(config)
            
            # Alinear profundidad con color
            self.align = rs.align(rs.stream.color)
            
            self.running = True
            self.thread = threading.Thread(target=self._capture_thread, daemon=True)
            self.thread.start()
            
            print("✓ Cámara RealSense iniciada")
            return True
        except Exception as e:
            print(f"Error al iniciar cámara: {e}")
            return False
    
    def stop(self):
        """Detener lectura de cámara"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.pipeline:
            self.pipeline.stop()
        print("✓ Cámara detenida")
    
    def _capture_thread(self):
        """Thread para captura de frames"""
        frame_count = 0
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                
                # Alinear frames
                aligned_frames = self.align.process(frames)
                
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                
                # Convertir a arrays
                rgb = np.asanyarray(color_frame.get_data())
                depth = np.asanyarray(depth_frame.get_data())
                
                timestamp_ms = int(time.time() * 1000)
                
                with self.lock:
                    self.rgb = rgb
                    self.depth = depth
                    self.timestamp = timestamp_ms
                    self.has_new_frame = True
                
                frame_count += 1
                if frame_count % 30 == 0:
                    print(f"  Cámara: {frame_count} frames capturados")
                    
            except Exception as e:
                print(f"Error en captura: {e}")
                time.sleep(0.1)
    
    def get_data(self):
        """Obtener últimos frames de la cámara"""
        with self.lock:
            has_new = self.has_new_frame
            self.has_new_frame = False
            return self.rgb, self.depth, self.timestamp, has_new


class DataSynchronizer:
    """Sincroniza datos de Lidar y Cámara"""
    
    def __init__(self, output_dir, time_tolerance_ms=50):
        self.output_dir = Path(output_dir)
        self.time_tolerance = time_tolerance_ms
        self.frame_count = 0
        self.create_directories()
        
    def create_directories(self):
        """Crear estructura de directorios"""
        self.pointcloud_dir = self.output_dir / "pointclouds"
        self.rgb_dir = self.output_dir / "rgb"
        self.depth_dir = self.output_dir / "depth"
        self.metadata_file = self.output_dir / "metadata.csv"
        
        self.pointcloud_dir.mkdir(parents=True, exist_ok=True)
        self.rgb_dir.mkdir(parents=True, exist_ok=True)
        self.depth_dir.mkdir(parents=True, exist_ok=True)
        
        # Crear archivo de metadatos
        with open(self.metadata_file, 'w') as f:
            f.write("frame_id,lidar_timestamp_ms,camera_timestamp_ms,time_diff_ms\n")
        
        print(f"✓ Directorios creados en: {self.output_dir}")
    
    def save_synchronized_data(self, points, rgb, depth, lidar_ts, camera_ts):
        """Guardar datos sincronizados"""
        frame_id = self.frame_count
        time_diff = abs(lidar_ts - camera_ts)
        
        try:
            # Guardar nube de puntos (NPY para Python)
            if points is not None and len(points) > 0:
                pcd_file = self.pointcloud_dir / f"{frame_id:06d}.npy"
                np.save(str(pcd_file), points)
            
            # Guardar imagen RGB
            if rgb is not None:
                rgb_file = self.rgb_dir / f"{frame_id:06d}.png"
                cv2.imwrite(str(rgb_file), rgb)
            
            # Guardar mapa de profundidad
            if depth is not None:
                depth_file = self.depth_dir / f"{frame_id:06d}.png"
                cv2.imwrite(str(depth_file), depth)
            
            # Guardar metadatos
            with open(self.metadata_file, 'a') as f:
                f.write(f"{frame_id:06d},{lidar_ts},{camera_ts},{time_diff}\n")
            
            if frame_id % 10 == 0:
                print(f"Frame {frame_id} guardado | Diff: {time_diff}ms")
            
            self.frame_count += 1
            return True
            
        except Exception as e:
            print(f"Error al guardar frame {frame_id}: {e}")
            return False


class LidarCameraFusion:
    """Programa principal de fusión"""
    
    def __init__(self, output_dir, lidar_ip='192.168.1.201', lidar_port=2368):
        self.output_dir = output_dir
        self.lidar_ip = lidar_ip
        self.lidar_port = lidar_port
        self.running = False
        
        self.lidar = LidarReader(lidar_ip, lidar_port)
        self.camera = CameraReader()
        self.synchronizer = DataSynchronizer(output_dir)
    
    def start(self):
        """Iniciar adquisición de datos"""
        print("\n" + "="*50)
        print("  Fusión Lidar-Cámara (Python)")
        print("="*50 + "\n")
        
        # Iniciar dispositivos
        self.lidar.start()
        if not self.camera.start():
            print("Error: No se puede iniciar la cámara")
            self.lidar.stop()
            return False
        
        self.running = True
        print("\n✓ Sistema iniciado. Presione Ctrl+C para salir.\n")
        
        start_time = time.time()
        frame_count = 0
        
        try:
            while self.running:
                # Obtener datos de Lidar
                points, lidar_ts, lidar_new = self.lidar.get_data()
                
                # Obtener datos de Cámara
                rgb, depth, camera_ts, camera_new = self.camera.get_data()
                
                # Sincronizar si ambos tienen datos nuevos
                if lidar_new and camera_new and points is not None and rgb is not None:
                    time_diff = abs(lidar_ts - camera_ts)
                    
                    if time_diff <= self.synchronizer.time_tolerance:
                        self.synchronizer.save_synchronized_data(
                            points, rgb, depth, lidar_ts, camera_ts
                        )
                        frame_count += 1
                
                # Mostrar estadísticas cada 100 frames
                if frame_count > 0 and frame_count % 100 == 0:
                    elapsed = time.time() - start_time
                    fps = frame_count / elapsed
                    print(f"FPS: {fps:.1f} | Frames: {frame_count}")
                
                time.sleep(0.01)  # Pequeña pausa para no saturar CPU
        
        except KeyboardInterrupt:
            print("\n\n¡Deteniendo...")
        
        finally:
            self.stop(frame_count, start_time)
    
    def stop(self, frame_count, start_time):
        """Detener sistema"""
        self.running = False
        
        self.lidar.stop()
        self.camera.stop()
        
        elapsed = time.time() - start_time
        avg_fps = frame_count / elapsed if elapsed > 0 else 0
        
        print("\n" + "="*50)
        print("  RESUMEN")
        print("="*50)
        print(f"Frames guardados: {frame_count}")
        print(f"Tiempo total: {elapsed:.1f}s")
        print(f"FPS promedio: {avg_fps:.1f}")
        print(f"Directorio: {self.output_dir}")
        print("="*50 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description='Fusión sincronizada de Lidar Velodyne y Cámara RealSense',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  python3 lidar_camera_fusion.py ~/datos_fusion
  python3 lidar_camera_fusion.py ~/datos_fusion --ip 192.168.1.201 --port 2368
        """
    )
    
    parser.add_argument('output_dir', help='Directorio de salida para los datos')
    parser.add_argument('--ip', type=str, default='192.168.1.201', help='IP del Lidar (default: 192.168.1.201)')
    parser.add_argument('--port', type=int, default=2368, help='Puerto del Lidar (default: 2368)')
    
    args = parser.parse_args()
    
    # Crear fusionador
    fusion = LidarCameraFusion(args.output_dir, args.ip, args.port)
    
    try:
        fusion.start()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
