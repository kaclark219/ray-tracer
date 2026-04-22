cmake_minimum_required(VERSION 3.24)
project(raytracer LANGUAGES CXX)

include(CheckLanguage)

option(RAYTRACER_ENABLE_CUDA "Build the CUDA renderer target" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(RAYTRACER_COMMON_SOURCES
    src/image/image_ppm.cpp
)

set(RAYTRACER_CPU_SOURCES
    src/main.cpp
    src/render_cpu.cpp
    ${RAYTRACER_COMMON_SOURCES}
)

set(RAYTRACER_CUDA_SOURCES
    src/main_cuda.cu
    src/render_cuda.cu
    ${RAYTRACER_COMMON_SOURCES}
)

add_executable(raytracer_cpu ${RAYTRACER_CPU_SOURCES})
target_include_directories(raytracer_cpu PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

if (MSVC)
    target_compile_options(raytracer_cpu PRIVATE /W4)
else()
    target_compile_options(raytracer_cpu PRIVATE -Wall -Wextra -Wpedantic)
endif()

if (RAYTRACER_ENABLE_CUDA)
    check_language(CUDA)
    if (CMAKE_CUDA_COMPILER)
        enable_language(CUDA)

        if (NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
            set(CMAKE_CUDA_ARCHITECTURES 86)
        endif()

        add_executable(raytracer_cuda ${RAYTRACER_CUDA_SOURCES})
        target_include_directories(raytracer_cuda PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

        set_target_properties(raytracer_cuda PROPERTIES
            CUDA_SEPARABLE_COMPILATION ON
            CUDA_STANDARD 17
            CUDA_STANDARD_REQUIRED ON
        )

        if (MSVC)
            target_compile_options(raytracer_cuda PRIVATE /W4)
        else()
            target_compile_options(raytracer_cuda PRIVATE -Wall -Wextra -Wpedantic)
        endif()
    else()
        message(WARNING "CUDA compiler not found; skipping raytracer_cuda target.")
    endif()
endif()

# to build from repo root:
#   cmake --fresh -S . -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build -j
#
# to run:
#   ./build/raytracer_cpu
#   ./build/raytracer_cuda
