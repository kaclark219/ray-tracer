
cmake_minimum_required(VERSION 3.24)
project(raytracer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(raytracer
    src/main.cpp
    src/render_cpu.cpp
    src/image/image_ppm.cpp
)

target_include_directories(raytracer PRIVATE src)

if (MSVC)
    target_compile_options(raytracer PRIVATE /W4)
else()
    target_compile_options(raytracer PRIVATE -Wall -Wextra -Wpedantic)
endif()

# to build:
    # cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    # cmake --build build -j
    # ./build/raytracer