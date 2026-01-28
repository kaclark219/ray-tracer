// Simple CPU renderer example: create a white image with a black diagonal line
#include "image/image.h"
#include <string>
#include <cstdlib>
#include <cmath>

// Bresenham line drawing between (x0,y0) and (x1,y1)
static void drawLine(Image &img, int x0, int y0, int x1, int y1, const Color &c) {
	int dx = std::abs(x1 - x0);
	int sx = x0 < x1 ? 1 : -1;
	int dy = -std::abs(y1 - y0);
	int sy = y0 < y1 ? 1 : -1;
	int err = dx + dy;

	while (true) {
		img.setPixel(x0, y0, c);
		if (x0 == x1 && y0 == y1) break;
		int e2 = 2 * err;
		if (e2 >= dy) { err += dy; x0 += sx; }
		if (e2 <= dx) { err += dx; y0 += sy; }
	}
}

int run_render_cpu_example() {
	const int W = 800;
	const int H = 600;
	Image img(W, H);

	// fill white
	Color white(1.0f, 1.0f, 1.0f);
	Color black(0.0f, 0.0f, 0.0f);
	for (int y = 0; y < H; ++y) {
		for (int x = 0; x < W; ++x) {
			img.setPixel(x, y, white);
		}
	}

	// draw a black diagonal line
	drawLine(img, 0, 0, W - 1, H - 1, black);

	std::string out = "output.ppm";
	if (!img.writePPM(out)) return 1;
	return 0;
}

