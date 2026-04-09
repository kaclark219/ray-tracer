#include "image/image.h"
#include "components/vec3.h"
#include "camera.h"
#include "components/point.h"
#include "components/ray.h"
#include "components/material.h"
#include "components/light.h"
#include "components/intersect_data.h"
#include "components/k-d_trees.h"
#include "components/photon_map.h"
#include "object.h"
#include "objects/sphere.h"
#include "objects/triangle.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

using std::make_unique;
using std::unique_ptr;
using std::vector;

const int W = 600;
const int H = 600;
const int MAX_DEPTH = 3;
const float EPSILON = 1e-2f;
const int GLOBAL_PHOTON_COUNT = 100000;
const float PHOTON_GATHER_RADIUS = 85.0f;
const float LIGHT_MIN_X = 213.0f;
const float LIGHT_MAX_X = 343.0f;
const float LIGHT_MIN_Z = 227.0f;
const float LIGHT_MAX_Z = 332.0f;
const float LIGHT_PANEL_Y = 548.79f;
const int AREA_LIGHT_SAMPLES_AXIS = 4;

static bool findNearestKDTree(
    const Ray& ray,
    const KDTree& tree,
    float& nearest,
    Object*& objHit
) {
    KDTree::HitResult result;
    if (!tree.intersect(ray, result) || result.object == nullptr || result.t <= EPSILON) {
        nearest = std::numeric_limits<float>::infinity();
        objHit = nullptr;
        return false;
    }

    nearest = result.t;
    objHit = const_cast<Object*>(result.object);
    return true;
}

static bool isLightPanelHit(const Point& p) {
    return p.getY() > 548.7f && p.getX() >= LIGHT_MIN_X && p.getX() <= LIGHT_MAX_X && p.getZ() >= LIGHT_MIN_Z && p.getZ() <= LIGHT_MAX_Z;
}

static Point pointAlongRay(const Ray& ray, float t) {
    const Vec3 direction = ray.getDirection();
    return Point(
        ray.getOrigin().getX() + direction.getX() * t,
        ray.getOrigin().getY() + direction.getY() * t,
        ray.getOrigin().getZ() + direction.getZ() * t
    );
}

static void encodeDirection(const Vec3& direction, float& phi, float& theta) {
    Vec3 dir = direction;
    dir.normalize();
    phi = std::atan2(dir.getZ(), dir.getX());
    theta = std::acos(std::max(-1.0f, std::min(1.0f, dir.getY())));
}

static Vec3 decodeDirection(float phi, float theta) {
    const float sinTheta = std::sin(theta);
    Vec3 direction(
        sinTheta * std::cos(phi),
        std::cos(theta),
        sinTheta * std::sin(phi)
    );
    direction.normalize();
    return direction;
}

static Color makeScaledColor(const Color& color, float scalar) {
    Color scaled;
    scaled.r = color.r * scalar;
    scaled.g = color.g * scalar;
    scaled.b = color.b * scalar;
    return scaled;
}

static Color makeColorFromFloats(float r, float g, float b) {
    Color color;
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
}

static Color modulateColor(const Color& a, const Color& b) {
    Color result;
    result.r = (a.r / 255.0f) * b.r;
    result.g = (a.g / 255.0f) * b.g;
    result.b = (a.b / 255.0f) * b.b;
    return result;
}

static void addColor(Color& target, const Color& value) {
    target.r += value.r;
    target.g += value.g;
    target.b += value.b;
}

static void addModulatedColor(Color& target, const Color& albedo, const Color& lightColor, float scale) {
    target.r += (albedo.r / 255.0f) * lightColor.r * scale;
    target.g += (albedo.g / 255.0f) * lightColor.g * scale;
    target.b += (albedo.b / 255.0f) * lightColor.b * scale;
}

static Vec3 sampleDiffuseHemisphere(const Vec3& normal, std::mt19937& rng) {
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    const float u1 = unit(rng);
    const float u2 = unit(rng);
    const float r = std::sqrt(u1);
    const float phi = 2.0f * 3.14159265358979323846f * u2;

    Vec3 w = normal;
    w.normalize();
    Vec3 tangent = (std::abs(w.getX()) > 0.1f) ? Vec3(0.0f, 1.0f, 0.0f).cross(w) : Vec3(1.0f, 0.0f, 0.0f).cross(w);
    tangent.normalize();
    Vec3 bitangent = w.cross(tangent);
    bitangent.normalize();

    const float x = r * std::cos(phi);
    const float z = r * std::sin(phi);
    const float y = std::sqrt(std::max(0.0f, 1.0f - u1));

    Vec3 sampled(
        tangent.getX() * x + w.getX() * y + bitangent.getX() * z,
        tangent.getY() * x + w.getY() * y + bitangent.getY() * z,
        tangent.getZ() * x + w.getZ() * y + bitangent.getZ() * z
    );
    sampled.normalize();
    return sampled;
}

static bool isDiffuseSurface(const Material& material) {
    return material.getReflectivity() < 0.5f;
}

static Ray makeReflectedRay(const Point& hitPoint, const Vec3& incident, const Vec3& normal) {
    Vec3 reflectDir = incident - (normal * (2.0f * incident.dot(normal)));
    reflectDir.normalize();

    const float offsetSign = (reflectDir.dot(normal) >= 0.0f) ? 1.0f : -1.0f;
    Point reflectOrigin(
        hitPoint.getX() + normal.getX() * EPSILON * offsetSign,
        hitPoint.getY() + normal.getY() * EPSILON * offsetSign,
        hitPoint.getZ() + normal.getZ() * EPSILON * offsetSign
    );
    return Ray(reflectOrigin, reflectDir);
}

static bool isLightSampleVisible(
    const Point& hitPoint,
    const Vec3& normal,
    const Point& lightSample,
    const KDTree& kdTree
) {
    Vec3 lightVector = lightSample - hitPoint;
    const float lightDistance = lightVector.length();
    if (lightDistance <= EPSILON) {
        return true;
    }

    lightVector.normalize();
    Point shadowOrigin(
        hitPoint.getX() + normal.getX() * EPSILON,
        hitPoint.getY() + normal.getY() * EPSILON,
        hitPoint.getZ() + normal.getZ() * EPSILON
    );
    Ray shadowRay(shadowOrigin, lightVector);

    KDTree::HitResult result;
    if (!kdTree.intersect(shadowRay, result) || result.object == nullptr) {
        return true;
    }

    if (result.t <= EPSILON) {
        return true;
    }

    if (result.t >= lightDistance - (EPSILON * 2.0f)) {
        return true;
    }

    const Point blocker = pointAlongRay(shadowRay, result.t);
    return isLightPanelHit(blocker);
}

static void tracePhoton(
    const Ray& ray,
    const KDTree& kdTree,
    PhotonMap& photonMap,
    const Color& photonPower,
    std::mt19937& rng,
    int depth
) {
    if (depth >= MAX_DEPTH) {
        return;
    }

    float nearest = std::numeric_limits<float>::infinity();
    Object* objHit = nullptr;
    if (!findNearestKDTree(ray, kdTree, nearest, objHit)) {
        return;
    }

    const Point hitPoint = pointAlongRay(ray, nearest);
    if (isLightPanelHit(hitPoint)) {
        return;
    }

    Vec3 normal = objHit->normal(hitPoint);
    normal.normalize();

    Vec3 incident = ray.getDirection();
    incident.normalize();
    if (normal.dot(incident) > 0.0f) {
        normal = normal * -1.0f;
    }

    const Material& material = objHit->getMaterial();
    if (isDiffuseSurface(material)) {
        float phi = 0.0f;
        float theta = 0.0f;
        encodeDirection(incident, phi, theta);
        photonMap.store(Photon(hitPoint, photonPower, phi, theta));
    }

    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    const float reflectivity = std::max(0.0f, std::min(1.0f, material.getReflectivity()));
    const Color diffuse = material.getDiffuse();
    const float diffuseStrength = std::min(0.95f, ((diffuse.r + diffuse.g + diffuse.b) / (3.0f * 255.0f)) * (1.0f - reflectivity));
    const float roulette = unit(rng);

    if (roulette < reflectivity) {
        tracePhoton(makeReflectedRay(hitPoint, incident, normal), kdTree, photonMap, photonPower, rng, depth + 1);
        return;
    }

    if (roulette < reflectivity + diffuseStrength) {
        const Vec3 bounceDir = sampleDiffuseHemisphere(normal, rng);
        const Point bounceOrigin(
            hitPoint.getX() + normal.getX() * EPSILON,
            hitPoint.getY() + normal.getY() * EPSILON,
            hitPoint.getZ() + normal.getZ() * EPSILON
        );
        tracePhoton(Ray(bounceOrigin, bounceDir), kdTree, photonMap, modulateColor(photonPower, diffuse), rng, depth + 1);
    }
}

static void emitGlobalPhotons(
    PhotonMap& photonMap,
    const KDTree& kdTree,
    int photonCount,
    const Color& lightColor,
    float lightIntensity
) {
    photonMap.clear();
    photonMap.reserve(static_cast<std::size_t>(photonCount));

    std::mt19937 rng(71101u);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);

    const Color photonPower = makeScaledColor(lightColor, lightIntensity / static_cast<float>(photonCount));

    for (int i = 0; i < photonCount; ++i) {
        const float x = LIGHT_MIN_X + (LIGHT_MAX_X - LIGHT_MIN_X) * unit(rng);
        const float z = LIGHT_MIN_Z + (LIGHT_MAX_Z - LIGHT_MIN_Z) * unit(rng);

        const float u1 = unit(rng);
        const float u2 = unit(rng);
        const float r = std::sqrt(u1);
        const float phi = 2.0f * 3.14159265358979323846f * u2;
        const float dx = r * std::cos(phi);
        const float dz = r * std::sin(phi);
        const float dy = -std::sqrt(std::max(0.0f, 1.0f - u1));

        Vec3 direction(dx, dy, dz);
        direction.normalize();

        tracePhoton(Ray(Point(x, LIGHT_PANEL_Y - EPSILON, z), direction), kdTree, photonMap, photonPower, rng, 0);
    }

    photonMap.build();
}

static Color estimateIndirectPhotonRadiance(
    const IntersectData& data,
    const PhotonMap& photonMap
) {
    const std::vector<const Photon*> nearby = photonMap.gather(data.hit_point, PHOTON_GATHER_RADIUS);
    if (nearby.empty()) {
        return Color(0, 0, 0);
    }

    Vec3 shadingNormal = data.normal;
    shadingNormal.normalize();
    if (shadingNormal.dot(data.incoming) > 0.0f) {
        shadingNormal = shadingNormal * -1.0f;
    }

    float fluxR = 0.0f;
    float fluxG = 0.0f;
    float fluxB = 0.0f;
    float totalWeight = 0.0f;
    const float radiusSquared = PHOTON_GATHER_RADIUS * PHOTON_GATHER_RADIUS;

    for (const Photon* photon : nearby) {
        const Vec3 incomingDir = decodeDirection(photon->phi, photon->theta);
        if (shadingNormal.dot(incomingDir * -1.0f) <= 0.0f) {
            continue;
        }

        const float dx = photon->position.getX() - data.hit_point.getX();
        const float dy = photon->position.getY() - data.hit_point.getY();
        const float dz = photon->position.getZ() - data.hit_point.getZ();
        const float distanceSquared = dx * dx + dy * dy + dz * dz;
        const float weight = std::max(0.0f, 1.0f - (distanceSquared / radiusSquared));
        if (weight <= 0.0f) {
            continue;
        }

        fluxR += photon->power.r * weight;
        fluxG += photon->power.g * weight;
        fluxB += photon->power.b * weight;
        totalWeight += weight;
    }

    if (totalWeight <= 0.0f) {
        return Color(0, 0, 0);
    }

    const float area = 3.14159265358979323846f * PHOTON_GATHER_RADIUS * PHOTON_GATHER_RADIUS;
    const Color diffuse = data.object->getMaterial().getDiffuse();

    const float indirectScale = 1200.0f;
    const float indirectR = (fluxR / area) * (diffuse.r / 255.0f) * indirectScale;
    const float indirectG = (fluxG / area) * (diffuse.g / 255.0f) * indirectScale;
    const float indirectB = (fluxB / area) * (diffuse.b / 255.0f) * indirectScale;

    Color result = makeColorFromFloats(indirectR, indirectG, indirectB);
    result.clamp();
    return result;
}

static void writePhotonMapImage(
    const Image& baseImage,
    const PhotonMap& photonMap,
    const Point& cameraPos,
    const Vec3& forward,
    const Vec3& right,
    const Vec3& up,
    float focalLength,
    float filmWidth,
    float filmHeight
) {
    Image photonImg = baseImage;
    const Color photonDotColor(255, 140, 0);

    for (const Photon& photon : photonMap.getPhotons()) {
        Vec3 toPhoton = photon.position - cameraPos;
        const float camX = toPhoton.dot(right);
        const float camY = toPhoton.dot(up);
        const float camZ = toPhoton.dot(forward);
        if (camZ <= 0.0f) {
            continue;
        }

        const float filmX = focalLength * camX / camZ;
        const float filmY = focalLength * camY / camZ;
        if (filmX < -filmWidth * 0.5f || filmX > filmWidth * 0.5f ||
            filmY < -filmHeight * 0.5f || filmY > filmHeight * 0.5f) {
            continue;
        }

        const int px = static_cast<int>(((filmX / filmWidth) + 0.5f) * static_cast<float>(W));
        const int py = static_cast<int>((0.5f - (filmY / filmHeight)) * static_cast<float>(H));

        for (int oy = -1; oy <= 1; ++oy) {
            for (int ox = -1; ox <= 1; ++ox) {
                if ((ox * ox) + (oy * oy) <= 1) {
                    photonImg.setPixel(px + ox, py + oy, photonDotColor);
                }
            }
        }
    }

    photonImg.writePPM("photon_map.ppm");
}

static Color shadePoint(
    const IntersectData& data,
    const vector<unique_ptr<Light>>& lights,
    const KDTree& kdTree,
    const PhotonMap& photonMap,
    const Color& ambientLight,
    const Vec3& viewDir
) {
    const Material& material = data.object->getMaterial();
    Color result;
    addModulatedColor(result, material.getAmbient(), ambientLight, 1.0f);
    const Color diffuseColor = material.getDiffuse();

    if (!lights.empty()) {
        const Color areaLightColor = makeScaledColor(lights.front()->getColor(), lights.front()->getIntensity());
        Vec3 normal = data.normal;
        if (normal.dot(viewDir) < 0.0f) {
            normal = normal * -1.0f;
        }

        const int totalSamples = AREA_LIGHT_SAMPLES_AXIS * AREA_LIGHT_SAMPLES_AXIS;
        for (int sy = 0; sy < AREA_LIGHT_SAMPLES_AXIS; ++sy) {
            for (int sx = 0; sx < AREA_LIGHT_SAMPLES_AXIS; ++sx) {
                const float tx = (static_cast<float>(sx) + 0.5f) / static_cast<float>(AREA_LIGHT_SAMPLES_AXIS);
                const float tz = (static_cast<float>(sy) + 0.5f) / static_cast<float>(AREA_LIGHT_SAMPLES_AXIS);
                const Point lightSample(
                    LIGHT_MIN_X + (LIGHT_MAX_X - LIGHT_MIN_X) * tx,
                    LIGHT_PANEL_Y,
                    LIGHT_MIN_Z + (LIGHT_MAX_Z - LIGHT_MIN_Z) * tz
                );

                Vec3 lightVector = lightSample - data.hit_point;
                lightVector.normalize();
                const float nDotL = std::max(normal.dot(lightVector), 0.0f);
                const Color sampleLight = makeScaledColor(areaLightColor, 1.0f / static_cast<float>(totalSamples));
                if (nDotL <= 0.0f) {
                    continue;
                }
                if (!isLightSampleVisible(data.hit_point, normal, lightSample, kdTree)) {
                    const float shadowBounce = 0.02f;
                    addModulatedColor(result, diffuseColor, sampleLight, nDotL * shadowBounce);
                    continue;
                }

                Vec3 reflection = (normal * (2.0f * nDotL)) - lightVector;
                reflection.normalize();
                const float specularFactor = std::pow(std::max(reflection.dot(viewDir), 0.0f), material.getShininess());

                addModulatedColor(result, diffuseColor, sampleLight, nDotL);
                addModulatedColor(result, material.getSpecular(), sampleLight, specularFactor);
            }
        }
    }

    addColor(result, estimateIndirectPhotonRadiance(data, photonMap));
    result.clamp();
    return result;
}

static Color traceRay(
    const Ray& ray,
    int depth,
    const vector<unique_ptr<Light>>& lights,
    const vector<unique_ptr<Object>>& objects,
    const KDTree& kdTree,
    const PhotonMap& photonMap,
    const Color& ambientLight,
    const Color& backgroundColor
) {
    if (depth >= MAX_DEPTH) {
        return backgroundColor;
    }

    float nearest = std::numeric_limits<float>::infinity();
    Object* objHit = nullptr;
    if (!findNearestKDTree(ray, kdTree, nearest, objHit)) {
        return backgroundColor;
    }

    Vec3 rayDir = ray.getDirection();
    const Point hitPoint(
        ray.getOrigin().getX() + rayDir.getX() * nearest,
        ray.getOrigin().getY() + rayDir.getY() * nearest,
        ray.getOrigin().getZ() + rayDir.getZ() * nearest
    );
    Vec3 normal = objHit->normal(hitPoint);
    normal.normalize();

    rayDir.normalize();
    Vec3 viewDir = rayDir * -1.0f;
    viewDir.normalize();

    IntersectData data;
    data.hit = true;
    data.object = objHit;
    data.t = nearest;
    data.hit_point = hitPoint;
    data.normal = normal;
    data.incoming = rayDir;

    if (isLightPanelHit(hitPoint)) {
        return Color(255, 220, 180);
    }

    Color localColor = shadePoint(data, lights, kdTree, photonMap, ambientLight, viewDir);

    const float reflectivity = objHit->getMaterial().getReflectivity();
    if (reflectivity > 0.0f) {
        const float dotIN = rayDir.dot(normal);
        Vec3 reflectDir = rayDir - (normal * (2.0f * dotIN));
        reflectDir.normalize();

        const float offsetSign = (reflectDir.dot(normal) >= 0.0f) ? 1.0f : -1.0f;
        Point reflectOrigin(
            hitPoint.getX() + normal.getX() * EPSILON * offsetSign,
            hitPoint.getY() + normal.getY() * EPSILON * offsetSign,
            hitPoint.getZ() + normal.getZ() * EPSILON * offsetSign
        );

        const Color reflected = traceRay(Ray(reflectOrigin, reflectDir), depth + 1, lights, objects, kdTree, photonMap, ambientLight, backgroundColor);
        localColor = localColor + (reflected * reflectivity);
        localColor.clamp();
    }

    return localColor;
}

static void addTriangle(vector<unique_ptr<Object>>& scene, const Point& a, const Point& b, const Point& c, const Material& material) {
    auto triangle = make_unique<Triangle>(a, b, c);
    triangle->setMaterial(material);
    scene.push_back(std::move(triangle));
}

static void addQuad(vector<unique_ptr<Object>>& scene, const Point& a, const Point& b, const Point& c, const Point& d, const Material& material) {
    addTriangle(scene, a, b, c, material);
    addTriangle(scene, a, c, d, material);
}

static void addSphere(vector<unique_ptr<Object>>& scene, const Point& center, float radius, const Material& material) {
    auto sphere = make_unique<Sphere>(center, radius);
    sphere->setMaterial(material);
    scene.push_back(std::move(sphere));
}

static Material tintedMatte(int r, int g, int b) {
    Material material = Material::Matte();
    material.setAmbient(Color(r / 4, g / 4, b / 4));
    material.setDiffuse(Color(r, g, b));
    material.setSpecular(Color(10, 10, 10));
    material.setShininess(10.0f);
    material.setReflectivity(0.0f);
    return material;
}

static Material ceilingMatte() {
    Material material = Material::Matte();
    material.setAmbient(Color(150, 138, 120));
    material.setDiffuse(Color(245, 232, 210));
    material.setSpecular(Color(10, 10, 10));
    material.setShininess(10.0f);
    material.setReflectivity(0.0f);
    return material;
}

static void buildCornellBoxScene(vector<unique_ptr<Object>>& scene) {
    const Material white = tintedMatte(210, 210, 210);
    const Material ceiling = ceilingMatte();
    const Material red = tintedMatte(190, 45, 35);
    const Material green = tintedMatte(55, 190, 70);
    const Material lightPanel(Color(255, 220, 180), Color(255, 220, 180), Color(0, 0, 0), 1.0f, 0.0f);
    const Material mirror = Material::Mirror();

    addQuad(
        scene,
        Point(552.8f, 0.0f, 0.0f),
        Point(0.0f, 0.0f, 0.0f),
        Point(0.0f, 0.0f, 559.2f),
        Point(549.6f, 0.0f, 559.2f),
        white
    );

    addQuad(
        scene,
        Point(556.0f, 548.8f, 0.0f),
        Point(556.0f, 548.8f, 559.2f),
        Point(343.0f, 548.8f, 559.2f),
        Point(343.0f, 548.8f, 0.0f),
        ceiling
    );
    addQuad(
        scene,
        Point(213.0f, 548.8f, 0.0f),
        Point(213.0f, 548.8f, 559.2f),
        Point(0.0f, 548.8f, 559.2f),
        Point(0.0f, 548.8f, 0.0f),
        ceiling
    );
    addQuad(
        scene,
        Point(343.0f, 548.8f, 0.0f),
        Point(343.0f, 548.8f, 227.0f),
        Point(213.0f, 548.8f, 227.0f),
        Point(213.0f, 548.8f, 0.0f),
        ceiling
    );
    addQuad(
        scene,
        Point(343.0f, 548.8f, 332.0f),
        Point(343.0f, 548.8f, 559.2f),
        Point(213.0f, 548.8f, 559.2f),
        Point(213.0f, 548.8f, 332.0f),
        ceiling
    );

    addQuad(
        scene,
        Point(549.6f, 0.0f, 559.2f),
        Point(0.0f, 0.0f, 559.2f),
        Point(0.0f, 548.8f, 559.2f),
        Point(556.0f, 548.8f, 559.2f),
        white
    );

    addQuad(
        scene,
        Point(0.0f, 0.0f, 559.2f),
        Point(0.0f, 0.0f, 0.0f),
        Point(0.0f, 548.8f, 0.0f),
        Point(0.0f, 548.8f, 559.2f),
        green
    );

    addQuad(
        scene,
        Point(552.8f, 0.0f, 0.0f),
        Point(549.6f, 0.0f, 559.2f),
        Point(556.0f, 548.8f, 559.2f),
        Point(556.0f, 548.8f, 0.0f),
        red
    );

    addQuad(
        scene,
        Point(343.0f, 548.79f, 227.0f),
        Point(343.0f, 548.79f, 332.0f),
        Point(213.0f, 548.79f, 332.0f),
        Point(213.0f, 548.79f, 227.0f),
        lightPanel
    );

    addSphere(scene, Point(186.0f, 82.0f, 169.0f), 82.0f, mirror);
    addSphere(scene, Point(369.0f, 160.0f, 351.0f), 160.0f, mirror);
}

int renderCPU() {
    vector<unique_ptr<Object>> scene;
    buildCornellBoxScene(scene);

    KDTree kdTree;
    kdTree.build(scene);

    vector<unique_ptr<Light>> lights;
    const Point lightCenter(278.0f, 548.5f, 279.5f);
    lights.push_back(make_unique<PointLight>(lightCenter, Color(255, 214, 170), 1.1f));

    PhotonMap globalPhotonMap;
    emitGlobalPhotons(globalPhotonMap, kdTree, GLOBAL_PHOTON_COUNT, Color(255, 214, 170), 1.1f);

    const Color ambientLight(70, 70, 70);
    const Color backgroundColor(0, 0, 0);
    Image img(W, H, backgroundColor);

    const Point cameraPos(278.0f, 273.0f, -800.0f);
    Vec3 forward(0.0f, 0.0f, 1.0f);
    Vec3 up(0.0f, 1.0f, 0.0f);
    forward.normalize();
    up.normalize();
    Vec3 right = forward.cross(up);
    right.normalize();
    up = right.cross(forward);
    up.normalize();

    const float focalLength = 35.0f;
    const float filmWidth = 25.0f;
    const float filmHeight = 25.0f;
    const Point filmCenter(
        cameraPos.getX() + forward.getX() * focalLength,
        cameraPos.getY() + forward.getY() * focalLength,
        cameraPos.getZ() + forward.getZ() * focalLength
    );

    const int samplesPerAxis = 2;
    const int samplesPerPixel = samplesPerAxis * samplesPerAxis;
    const auto renderStart = std::chrono::high_resolution_clock::now();

    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            float accumR = 0.0f;
            float accumG = 0.0f;
            float accumB = 0.0f;

            for (int sy = 0; sy < samplesPerAxis; ++sy) {
                for (int sx = 0; sx < samplesPerAxis; ++sx) {
                    const float u = (i + (sx + 0.5f) / samplesPerAxis) / static_cast<float>(W);
                    const float v = (j + (sy + 0.5f) / samplesPerAxis) / static_cast<float>(H);

                    const float filmX = (u - 0.5f) * filmWidth;
                    const float filmY = (0.5f - v) * filmHeight;

                    Point filmPoint(
                        filmCenter.getX() + right.getX() * filmX + up.getX() * filmY,
                        filmCenter.getY() + right.getY() * filmX + up.getY() * filmY,
                        filmCenter.getZ() + right.getZ() * filmX + up.getZ() * filmY
                    );

                    Vec3 rayDir = filmPoint - cameraPos;
                    rayDir.normalize();

                    const Color sampleColor = traceRay(
                        Ray(cameraPos, rayDir),
                        0,
                        lights,
                        scene,
                        kdTree,
                        globalPhotonMap,
                        ambientLight,
                        backgroundColor
                    );

                    accumR += sampleColor.r;
                    accumG += sampleColor.g;
                    accumB += sampleColor.b;
                }
            }

            const float invSamples = 1.0f / static_cast<float>(samplesPerPixel);
            Color finalColor(
                static_cast<int>(accumR * invSamples),
                static_cast<int>(accumG * invSamples),
                static_cast<int>(accumB * invSamples)
            );
            finalColor.clamp();
            img.setPixel(i, j, finalColor);
        }
    }

    const std::string out = "output_img.ppm";
    if (!img.writePPM(out)) {
        return 1;
    }

    const auto renderEnd = std::chrono::high_resolution_clock::now();
    const double renderMilliseconds = std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();
    std::cout << "Render time for output_img.ppm: " << renderMilliseconds << " ms\n";

    writePhotonMapImage(img, globalPhotonMap, cameraPos, forward, right, up, focalLength, filmWidth, filmHeight);
    return 0;
}
