#!/usr/bin/env python3
"""
Script de utilidad para procesar y visualizar datos capturados
Uso: python3 process_data.py <data_directory>
"""

import os
import sys
import cv2
import numpy as np
import pandas as pd
from pathlib import Path

try:
    import open3d as o3d
except ImportError:
    print("Advertencia: open3d no instalado. Algunas funciones no estarán disponibles.")
    o3d = None

def load_metadata(data_dir):
    """Cargar archivo de metadatos."""
    metadata_file = os.path.join(data_dir, "metadata.csv")
    if os.path.exists(metadata_file):
        return pd.read_csv(metadata_file)
    return None

def load_pointcloud(pcd_file):
    """Cargar nube de puntos PCD."""
    if o3d is None:
        print("Error: open3d no instalado")
        return None
    return o3d.io.read_point_cloud(pcd_file)

def load_rgb(rgb_file):
    """Cargar imagen RGB."""
    return cv2.imread(rgb_file)

def load_depth(depth_file):
    """Cargar mapa de profundidad."""
    return cv2.imread(depth_file, cv2.IMREAD_ANYDEPTH)

def get_statistics(data_dir):
    """Obtener estadísticas de los datos capturados."""
    print("\n" + "="*50)
    print("  Estadísticas de Datos Capturados")
    print("="*50 + "\n")
    
    # Contar archivos
    pc_dir = os.path.join(data_dir, "pointclouds")
    rgb_dir = os.path.join(data_dir, "rgb")
    depth_dir = os.path.join(data_dir, "depth")
    
    pc_count = len([f for f in os.listdir(pc_dir) if f.endswith('.pcd')])
    rgb_count = len([f for f in os.listdir(rgb_dir) if f.endswith('.png')])
    depth_count = len([f for f in os.listdir(depth_dir) if f.endswith('.png')])
    
    print(f"Nubes de puntos (PCD): {pc_count}")
    print(f"Imágenes RGB: {rgb_count}")
    print(f"Mapas de profundidad: {depth_count}")
    
    # Cargar metadatos
    metadata = load_metadata(data_dir)
    if metadata is not None:
        print(f"\nTotal de frames sincronizados: {len(metadata)}")
        print(f"Diferencia promedio de tiempo: {metadata['time_diff_ms'].mean():.2f} ms")
        print(f"Diferencia máxima de tiempo: {metadata['time_diff_ms'].max():.2f} ms")
        print(f"Diferencia mínima de tiempo: {metadata['time_diff_ms'].min():.2f} ms")
    
    # Tamaño de archivos
    pc_sizes = [os.path.getsize(os.path.join(pc_dir, f)) 
                for f in os.listdir(pc_dir) if f.endswith('.pcd')]
    rgb_sizes = [os.path.getsize(os.path.join(rgb_dir, f)) 
                 for f in os.listdir(rgb_dir) if f.endswith('.png')]
    
    if pc_sizes:
        print(f"\nTamaño promedio PCD: {np.mean(pc_sizes) / (1024*1024):.2f} MB")
    if rgb_sizes:
        print(f"Tamaño promedio RGB: {np.mean(rgb_sizes) / 1024:.2f} KB")
    
    total_size = sum(pc_sizes) + sum(rgb_sizes) + sum([
        os.path.getsize(os.path.join(depth_dir, f)) 
        for f in os.listdir(depth_dir) if f.endswith('.png')])
    
    print(f"\nTamaño total de datos: {total_size / (1024*1024):.2f} MB")
    print("\n" + "="*50 + "\n")

def visualize_frame(data_dir, frame_id):
    """Visualizar un frame específico."""
    pc_file = os.path.join(data_dir, "pointclouds", f"{frame_id:06d}.pcd")
    rgb_file = os.path.join(data_dir, "rgb", f"{frame_id:06d}.png")
    depth_file = os.path.join(data_dir, "depth", f"{frame_id:06d}.png")
    
    print(f"\nVisualizando frame {frame_id}...")
    
    # Mostrar imagen RGB
    if os.path.exists(rgb_file):
        rgb = load_rgb(rgb_file)
        cv2.imshow(f"RGB {frame_id}", rgb)
        print(f"RGB mostrado. Presione cualquier tecla para continuar.")
    
    # Mostrar mapa de profundidad
    if os.path.exists(depth_file):
        depth = load_depth(depth_file)
        # Normalizar para visualización
        depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.imshow(f"Profundidad {frame_id}", depth_normalized)
    
    # Mostrar nube de puntos
    if os.path.exists(pc_file) and o3d:
        pcd = load_pointcloud(pc_file)
        print(f"Nube de puntos con {len(pcd.points)} puntos")
        print("Presione 'q' para cerrar la visualización")
        o3d.visualization.draw_geometries([pcd])
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    if len(sys.argv) < 2:
        print("Uso: python3 process_data.py <data_directory> [opciones]")
        print("\nOpciones:")
        print("  stats              - Mostrar estadísticas")
        print("  visualize <id>     - Visualizar frame específico")
        print("  list               - Listar archivos disponibles")
        sys.exit(1)
    
    data_dir = sys.argv[1]
    
    if not os.path.exists(data_dir):
        print(f"Error: Directorio no encontrado: {data_dir}")
        sys.exit(1)
    
    # Comando por defecto: mostrar estadísticas
    if len(sys.argv) < 3:
        get_statistics(data_dir)
    else:
        command = sys.argv[2]
        
        if command == "stats":
            get_statistics(data_dir)
        
        elif command == "visualize" and len(sys.argv) > 3:
            frame_id = int(sys.argv[3])
            visualize_frame(data_dir, frame_id)
        
        elif command == "list":
            print("\nArchivos capturados:")
            for subdir in ["pointclouds", "rgb", "depth"]:
                dir_path = os.path.join(data_dir, subdir)
                if os.path.exists(dir_path):
                    files = sorted([f for f in os.listdir(dir_path)])
                    print(f"\n{subdir}/ ({len(files)} archivos)")
                    for f in files[:10]:
                        print(f"  {f}")
                    if len(files) > 10:
                        print(f"  ... y {len(files) - 10} más")

if __name__ == "__main__":
    main()
