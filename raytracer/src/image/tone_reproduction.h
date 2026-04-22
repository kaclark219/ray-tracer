#ifndef RAYTRACER_TONE_REPRODUCTION_H
#define RAYTRACER_TONE_REPRODUCTION_H

#include "../components/color.h"
#include <cmath>
#include <vector>
using std::vector;
using std::log;
using std::exp;

inline constexpr float ld_max = 1000.0f; // ld_max = maximum display luminance

// shared for tone reproduction operators
// L = 0.27R + 0.67G + 0.06B
inline float illuminance(const Color c) {
    return (0.27f * c.r + 0.67f * c.g + 0.06f * c.b);
}

// lal = exp((1/N)*SUM(log(delta + L(x,y))))
inline float logAvgLuminance(const vector<Color>& image) {
    if (image.empty()) {
        return 0.0f;
    }

    float sumLogL = 0.0f;
    const float delta = 1e-4f; // small value to avoid log(0)

    for (const auto& pixel : image) {
        float L = std::max(0.0f, illuminance(pixel));
        sumLogL += log(delta + L);
    }

    return exp(sumLogL / static_cast<float>(image.size()));
}


// PERCEPTUAL TONE REPRODUCTION
// sf = [(1.219+(ld-max/2)^0.4)/(1.219+(L_wa)^0.4)]^2.5
inline float scaleFactor(float L_wa) {
    if (L_wa <= 0.0f) {
        return 0.0f;
    }

    float numerator = 1.219f + std::pow(ld_max / 2.0f, 0.4f);
    float denominator = 1.219f + std::pow(L_wa, 0.4f);
    return std::pow(numerator / denominator, 2.5f) / ld_max;
}

// 1. Calculate sf by setting Lwa = L
// 2. Final display colors (Rtarget, Gtarget, Btarget) are the results of applying the sf to the R, G, B.
inline Color wardToneReproduction(const Color c, float L_wa) {
    float sf = scaleFactor(L_wa);
    return c * sf;
}


// PHOTOGRAPHIC TONE REPRODUCTION
inline constexpr float a = 0.18f; // % grey for zone 5

inline Color reinhardToneReproduction(const Color c, float L_wa) {
    if (L_wa <= 0.0f) {
        return Color();
    }

    // create scaled luminance values R s , G s , B s by mapping the key value to Zone V (18% gray)
    Color RGB_s = c * (a / L_wa);
    // find the reflectance for R r, G r, B r, based on film-like response
    Color RGB_r;
    RGB_r.r = RGB_s.r / (1.0f + RGB_s.r);
    RGB_r.g = RGB_s.g / (1.0f + RGB_s.g);
    RGB_r.b = RGB_s.b / (1.0f + RGB_s.b);
    // calculate target display luminance by simulating illumination .. assume that illuminant is “white” with luminance Ldmax
    Color RGB_t = RGB_r * ld_max;
    return Color(RGB_t.r / ld_max, RGB_t.g / ld_max, RGB_t.b / ld_max);
}

#endif // RAYTRACER_TONE_REPRODUCTION_H
