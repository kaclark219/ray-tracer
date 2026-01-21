cmake_minimum_required(VERSION 3.24)
project(raytracer LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

add_executable(raytracer
    src/main.cpp
    src/render_cpu.cpp
    src/render_cuda.cu
    src/image/image_ppm.cpp
)

target_include_directories(raytracer PRIVATE src)

set_target_properties(raytracer PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)

if (MSVC)
    target_compile_options(raytracer PRIVATE /W4)
else()
    target_compile_options(raytracer PRIVATE -Wall -Wextra -Wpedantic)
endif()

# to build:
    # cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    # cmake --build build -j
    # ./build/raytracer