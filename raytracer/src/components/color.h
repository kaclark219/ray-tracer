#ifndef RAYTRACER_COLOR_H
#define RAYTRACER_COLOR_H

#ifdef __CUDACC__
    #define CUDA_CALLABLE __host__ __device__
#else
    #define CUDA_CALLABLE
#endif

class Color {
    public:
        int r, g, b;

        // default constructor
        CUDA_CALLABLE Color() : r(0), g(0), b(0) {}

        // three parameter constructor
        CUDA_CALLABLE Color(int red, int green, int blue)
            : r(red), g(green), b(blue) {}

        // use default copy constructor/assignment
        CUDA_CALLABLE Color(const Color &c) = default;
        CUDA_CALLABLE Color& operator=(const Color &c) = default;
};

#undef CUDA_CALLABLE

#endif // RAYTRACER_COLOR_H