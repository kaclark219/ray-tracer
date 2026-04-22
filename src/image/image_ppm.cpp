#include "image.h"
#include <fstream>
#include <cmath>
#include "tone_reproduction.h"

static inline unsigned char clampU8(int v) {
    if (v <= 0) return 0;
    if (v >= 255) return 255;
    return static_cast<unsigned char>(v);
}

static inline unsigned char clampUnitToU8(float v) {
    if (v <= 0.0f) return 0;
    if (v >= 1.0f) return 255;
    return static_cast<unsigned char>(std::lround(v * 255.0f));
}

bool Image::writePPM(const std::string &filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) return false;

    ofs << "P6\n" << width << " " << height << "\n255\n";

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const Color &c = pixels[y * width + x];
            unsigned char rgb[3] = { clampU8(c.r), clampU8(c.g), clampU8(c.b) };
            ofs.write(reinterpret_cast<char *>(rgb), 3);
        }
    }

    return true;
}

bool Image::applyToneRepoduction(const std::string &filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) return false;

    ofs << "P6\n" << width << " " << height << "\n255\n";

    const float L_wa = logAvgLuminance(pixels);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const Color &c = pixels[y * width + x];
            Color toneMappedColor = wardToneReproduction(c, L_wa);
            // Color toneMappedColor = reinhardToneReproduction(c, L_wa);
            unsigned char rgb[3] = { clampUnitToU8(toneMappedColor.r), clampUnitToU8(toneMappedColor.g), clampUnitToU8(toneMappedColor.b) };
            ofs.write(reinterpret_cast<char *>(rgb), 3);
        }
    }
    return true;
}
