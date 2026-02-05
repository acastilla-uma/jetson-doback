cmake_minimum_required(VERSION 3.10)
project(lidar_camera_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Encontrar paquetes necesarios
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Incluir directorios
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIR}
)

# Agregar definiciones de PCL
add_definitions(${PCL_DEFINITIONS})

# Crear ejecutable principal
add_executable(lidar_camera_fusion 
    src/main.cpp
    src/lidar_reader.cpp
    src/camera_reader.cpp
    src/data_synchronizer.cpp
)

# Enlazar bibliotecas
target_link_libraries(lidar_camera_fusion
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${realsense2_LIBRARY}
    ${Boost_LIBRARIES}
    pthread
)

# Enlace condicional para <filesystem> en compiladores más viejos (libstdc++fs)
find_library(STDCXX_FS NAMES stdc++fs)
if(STDCXX_FS)
    message(STATUS "Linking with ${STDCXX_FS} to support std::filesystem/experimental::filesystem")
    target_link_libraries(lidar_camera_fusion PRIVATE ${STDCXX_FS})
endif()

# Opción para compilación optimizada en Jetson
if(JETSON_BUILD)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
endif()

# Crear ejecutable para detectar Lidar
add_executable(detect_lidar
    src/detect_lidar.cpp
)

# El detect_lidar solo necesita librerías estándar de red
target_link_libraries(detect_lidar
    pthread
)
)
