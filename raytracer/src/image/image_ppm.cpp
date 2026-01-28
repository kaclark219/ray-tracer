#include "image.h"
#include <fstream>
#include <cmath>

static inline unsigned char to_u8(float v) {
    if (v <= 0.0f) return 0;
    if (v >= 1.0f) return 255;
    return static_cast<unsigned char>(std::round(v * 255.0f));
}

bool Image::writePPM(const std::string &filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) return false;

    ofs << "P6\n" << width << " " << height << "\n255\n";

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const Color &c = pixels[y * width + x];
            unsigned char rgb[3] = { to_u8(c.r), to_u8(c.g), to_u8(c.b) };
            ofs.write(reinterpret_cast<char *>(rgb), 3);
        }
    }

    return true;
}
