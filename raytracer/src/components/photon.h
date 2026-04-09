#ifndef RAYTRACER_PHOTON_H
#define RAYTRACER_PHOTON_H

#include "point.h"
#include "color.h"

struct Photon {
    Point position;
    Color power;
    float phi;
    float theta;

    Photon(): position(), power(), phi(0.0f), theta(0.0f) {}

    Photon(const Point& pos, const Color& photonPower, float pphi, float ptheta): 
        position(pos), power(photonPower), phi(pphi), theta(ptheta) {}
};

#endif // RAYTRACER_PHOTON_H
