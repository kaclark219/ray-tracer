#include "image/image.h"
#include "components/vec3.h"
#include "camera.h"
#include "components/point.h"
#include "components/ray.h"
#include "components/material.h"
#include "components/light.h"
#include "components/intersect_data.h"
#include "components/k-d_trees.h"
#include "object.h"
#include "objects/sphere.h"
#include "objects/triangle.h"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using std::make_unique;
using std::unique_ptr;
using std::vector;

const int W = 600;
const int H = 600;
const int MAX_DEPTH = 3;
const float EPSILON = 1e-2f;

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

static Color shadePoint(
    const IntersectData& data,
    const vector<unique_ptr<Light>>& lights,
    const KDTree& kdTree,
    const Color& ambientLight,
    const Vec3& viewDir
) {
    Color result = data.object->getMaterial().getAmbient() * ambientLight;
    const Color diffuseColor = data.object->getMaterial().getDiffuse();

    for (const auto& light : lights) {
        Vec3 lightVector = light->getPosition() - data.hit_point;
        const float lightDistance = lightVector.length();
        lightVector.normalize();

        Vec3 normal = data.normal;
        if (normal.dot(viewDir) < 0.0f) {
            normal = normal * -1.0f;
        }

        Point shadowOrigin(
            data.hit_point.getX() + normal.getX() * EPSILON,
            data.hit_point.getY() + normal.getY() * EPSILON,
            data.hit_point.getZ() + normal.getZ() * EPSILON
        );
        Ray shadowRay(shadowOrigin, lightVector);
        const float nDotL = std::max(normal.dot(lightVector), 0.0f);
        const Color lightColor = light->getColor() * light->getIntensity();
        if (kdTree.occluded(shadowRay, lightDistance)) {
            const float shadowBounce = 0.10f;
            result = result + (diffuseColor * lightColor * nDotL * shadowBounce);
            continue;
        }

        Vec3 reflection = (normal * (2.0f * nDotL)) - lightVector;
        reflection.normalize();
        const float specularFactor = std::pow(std::max(reflection.dot(viewDir), 0.0f), data.object->getMaterial().getShininess());

        result = result + (diffuseColor * lightColor * nDotL);
        result = result + (data.object->getMaterial().getSpecular() * lightColor * specularFactor);
    }

    result.clamp();
    return result;
}

static Color traceRay(
    const Ray& ray,
    int depth,
    const vector<unique_ptr<Light>>& lights,
    const vector<unique_ptr<Object>>& objects,
    const KDTree& kdTree,
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

    if (hitPoint.getY() > 548.7f &&
        hitPoint.getX() >= 213.0f && hitPoint.getX() <= 343.0f &&
        hitPoint.getZ() >= 227.0f && hitPoint.getZ() <= 332.0f) {
        return Color(255, 220, 180);
    }

    Color localColor = shadePoint(data, lights, kdTree, ambientLight, viewDir);

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

        const Color reflected = traceRay(Ray(reflectOrigin, reflectDir), depth + 1, lights, objects, kdTree, ambientLight, backgroundColor);
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

    const Color ambientLight(90, 90, 90);
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
    return 0;
}
