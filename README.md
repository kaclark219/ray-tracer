# CSCI-711-01 Ray Tracer
**A fully custom-built ray tracing system in C++ & CUDA.** <br /> <br />
Created for CSCI-711 (Global Illumination) at RIT spring 2026, this project explores the ray tracing pipeline for lighting and rendering techniques.

## Table of Contents
- [Project Overview](#project-overview)
- [Dev Setup](#dev-setup)
  - [System Requirements](#system-requirements)
  - [Cloning the Repository](#cloning-the-repository)
  - [Installing Dependencies](#installing-dependencies)
- [Project Structure Overview](#project-structure-overview)
- [Building & Running the System](#building--running-the-system)
  - [Building](#building)
  - [Executing](#executing)

## Project Overview
This project implements a custom ray tracer with both CPU and CUDA render paths. The renderer builds a small scene from custom geometry, materials, lighting, and image output code rather than depending on an external rendering framework.

The current implementation includes:
- two spheres
- a floor made from triangles
- Phong-style local illumination
- recursive reflection and refraction
- procedural checkerboard texturing
- tone-mapped PPM image output

The renderer uses a right-handed coordinate system with `y` as the up axis. Scene data such as camera placement, object dimensions, and light placement comes from the course notes in `setup-artifacts/specifications.txt`.

## Dev Setup
### System Requirements
- **Operating System:** Linux is the primary tested environment for this repository.
- **Software Needed:**
  - CMake 3.24 or newer
  - A C++20-capable compiler such as `g++`
  - NVIDIA CUDA toolkit if you want to build the GPU renderer

Check installed versions:
```bash
cmake --version
g++ --version
nvcc --version
```

Example versions used while validating this repository:
- CMake `3.28.3`
- `g++` `13.3.0`
- CUDA `12.0`

### Cloning the Repository
```bash
git clone https://github.com/kaclark219/ray-tracer.git
cd ray-tracer
```

### Installing Dependencies
On Ubuntu or Debian-based systems, a typical base setup is:
```bash
sudo apt update
sudo apt install build-essential cmake
```
If you want CUDA support, install the NVIDIA driver and CUDA toolkit separately, then verify that `nvcc --version` works before configuring the project.

## Project Structure Overview
- `src/`: main renderer source tree
- `src/components/`: math types, materials, lights, and shared illumination/intersection helpers
- `src/objects/`: geometric primitives such as spheres and triangles
- `src/textures/`: procedural textures including checkerboard and noise helpers
- `src/image/`: image storage, PPM writing, and tone reproduction
- `src/main.cpp`: CPU renderer entry point
- `src/main_cuda.cu`: CUDA renderer entry point
- `build/`: local CMake build output
- `setup-artifacts/`: assignment notes, mock images, and reference material

## Building & Running the System
### Building
From the repository root:
```bash
cmake --fresh -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

If you are on a machine without CUDA, configure the project with CUDA disabled:
```bash
cmake --fresh -S . -B build -DCMAKE_BUILD_TYPE=Release -DRAYTRACER_ENABLE_CUDA=OFF
cmake --build build -j
```

### Executing
Run the CPU renderer with:
```bash
./build/raytracer_cpu
```

Run the CUDA renderer with:
```bash
./build/raytracer_cuda
```

The renderer writes image output files into the repository root, including `output_img_tonemapped.ppm`.
