#!/usr/bin/env python3
"""
Script para procesar y analizar datos capturados por lidar_camera_fusion.py
"""

import os
import sys
import cv2
import numpy as np
import pandas as pd
import argparse
from pathlib import Path
from collections import defaultdict

class DataProcessor:
    def __init__(self, data_dir):
        self.data_dir = Path(data_dir)
        self.pc_dir = self.data_dir / "pointclouds"
        self.rgb_dir = self.data_dir / "rgb"
        self.depth_dir = self.data_dir / "depth"
        self.metadata_file = self.data_dir / "metadata.csv"
    
    def get_statistics(self):
        """Obtener estadísticas de los datos capturados"""
        print("\n" + "="*50)
        print("  ESTADÍSTICAS DE DATOS CAPTURADOS")
        print("="*50 + "\n")
        
        # Contar archivos
        pc_files = list(self.pc_dir.glob("*.npy"))
        rgb_files = list(self.rgb_dir.glob("*.png"))
        depth_files = list(self.depth_dir.glob("*.png"))
        
        print(f"Nubes de puntos (NPY): {len(pc_files)}")
        print(f"Imágenes RGB: {len(rgb_files)}")
        print(f"Mapas de profundidad: {len(depth_files)}")
        
        # Cargar metadatos
        if self.metadata_file.exists():
            metadata = pd.read_csv(self.metadata_file)
            print(f"\nFrames sincronizados: {len(metadata)}")
            print(f"Diferencia temporal promedio: {metadata['time_diff_ms'].mean():.2f}ms")
            print(f"Diferencia temporal máxima: {metadata['time_diff_ms'].max():.2f}ms")
            print(f"Diferencia temporal mínima: {metadata['time_diff_ms'].min():.2f}ms")
        
        # Tamaños de archivos
        total_size = 0
        if pc_files:
            pc_sizes = [f.stat().st_size for f in pc_files]
            print(f"\nTamaño promedio PCD: {np.mean(pc_sizes) / 1024:.2f} KB")
            total_size += sum(pc_sizes)
        
        if rgb_files:
            rgb_sizes = [f.stat().st_size for f in rgb_files]
            print(f"Tamaño promedio RGB: {np.mean(rgb_sizes) / 1024:.2f} KB")
            total_size += sum(rgb_sizes)
        
        if depth_files:
            depth_sizes = [f.stat().st_size for f in depth_files]
            print(f"Tamaño promedio Depth: {np.mean(depth_sizes) / 1024:.2f} KB")
            total_size += sum(depth_sizes)
        
        print(f"\nTamaño total de datos: {total_size / (1024*1024):.2f} MB")
        print("="*50 + "\n")
    
    def visualize_frame(self, frame_id):
        """Visualizar un frame específico"""
        print(f"\nVisualizando frame {frame_id:06d}...\n")
        
        # Cargar RGB
        rgb_file = self.rgb_dir / f"{frame_id:06d}.png"
        if rgb_file.exists():
            rgb = cv2.imread(str(rgb_file))
            if rgb is not None:
                cv2.imshow(f'RGB Frame {frame_id:06d}', rgb)
                print("✓ RGB mostrado. Presione una tecla para continuar.")
                cv2.waitKey(0)
        
        # Cargar profundidad
        depth_file = self.depth_dir / f"{frame_id:06d}.png"
        if depth_file.exists():
            depth = cv2.imread(str(depth_file), cv2.IMREAD_ANYDEPTH)
            if depth is not None:
                # Normalizar para visualización
                depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                cv2.imshow(f'Depth Frame {frame_id:06d}', depth_norm)
                print("✓ Mapa de profundidad mostrado. Presione una tecla para continuar.")
                cv2.waitKey(0)
        
        # Cargar nube de puntos
        pc_file = self.pc_dir / f"{frame_id:06d}.npy"
        if pc_file.exists():
            try:
                points = np.load(str(pc_file))
                print(f"✓ Nube de puntos con {len(points)} puntos")
                print(f"  Rango X: {points[:,0].min():.2f} - {points[:,0].max():.2f}")
                print(f"  Rango Y: {points[:,1].min():.2f} - {points[:,1].max():.2f}")
                print(f"  Rango Z: {points[:,2].min():.2f} - {points[:,2].max():.2f}")
            except Exception as e:
                print(f"Error al cargar nube de puntos: {e}")
        
        cv2.destroyAllWindows()
    
    def list_files(self, limit=10):
        """Listar archivos capturados"""
        print("\nArchivos capturados:")
        
        for subdir_name, subdir_path in [("pointclouds", self.pc_dir), 
                                         ("rgb", self.rgb_dir), 
                                         ("depth", self.depth_dir)]:
            if subdir_path.exists():
                files = sorted([f for f in subdir_path.iterdir()])
                print(f"\n{subdir_name}/ ({len(files)} archivos)")
                for f in files[:limit]:
                    size = f.stat().st_size / 1024  # KB
                    print(f"  {f.name:20s} {size:10.2f} KB")
                if len(files) > limit:
                    print(f"  ... y {len(files) - limit} más")
    
    def convert_to_ply(self, frame_id, output_file=None):
        """Convertir nube de puntos a formato PLY"""
        try:
            import ply
        except ImportError:
            print("Error: librería ply no instalada")
            print("Instalar con: pip3 install ply")
            return False
        
        pc_file = self.pc_dir / f"{frame_id:06d}.npy"
        if not pc_file.exists():
            print(f"Error: No existe {pc_file}")
            return False
        
        points = np.load(str(pc_file))
        
        if output_file is None:
            output_file = self.pc_dir / f"{frame_id:06d}.ply"
        
        try:
            # Crear archivo PLY simple
            with open(output_file, 'w') as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(points)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("end_header\n")
                
                for point in points:
                    f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
            
            print(f"✓ Archivo PLY guardado: {output_file}")
            return True
        except Exception as e:
            print(f"Error al crear PLY: {e}")
            return False
    
    def extract_frame_range(self, start_id, end_id, output_dir):
        """Extraer rango de frames a nuevo directorio"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        (output_path / "pointclouds").mkdir(exist_ok=True)
        (output_path / "rgb").mkdir(exist_ok=True)
        (output_path / "depth").mkdir(exist_ok=True)
        
        count = 0
        for frame_id in range(start_id, end_id + 1):
            # Copiar archivos
            for src_dir, dst_dir in [(self.pc_dir, output_path / "pointclouds"),
                                      (self.rgb_dir, output_path / "rgb"),
                                      (self.depth_dir, output_path / "depth")]:
                src_file = src_dir / f"{frame_id:06d}"
                for src in src_dir.glob(f"{frame_id:06d}.*"):
                    import shutil
                    shutil.copy(str(src), str(dst_dir / src.name))
                    count += 1
        
        print(f"✓ {count} archivos extraídos a {output_dir}")

def main():
    parser = argparse.ArgumentParser(
        description='Procesar datos capturados por lidar_camera_fusion.py',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  python3 process_data.py ~/datos_fusion stats
  python3 process_data.py ~/datos_fusion list
  python3 process_data.py ~/datos_fusion visualize 0
  python3 process_data.py ~/datos_fusion ply 0
        """
    )
    
    parser.add_argument('data_dir', help='Directorio de datos')
    parser.add_argument('command', nargs='?', default='stats',
                       help='Comando: stats, list, visualize <id>, ply <id>')
    parser.add_argument('arg', nargs='?', default='0', help='Argumento (frame_id para visualize/ply)')
    
    args = parser.parse_args()
    
    processor = DataProcessor(args.data_dir)
    
    if args.command == 'stats':
        processor.get_statistics()
    elif args.command == 'list':
        processor.list_files()
    elif args.command == 'visualize':
        frame_id = int(args.arg)
        processor.visualize_frame(frame_id)
    elif args.command == 'ply':
        frame_id = int(args.arg)
        processor.convert_to_ply(frame_id)
    else:
        parser.print_help()

if __name__ == '__main__':
    main()
