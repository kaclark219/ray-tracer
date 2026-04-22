#include "image/image.h"
#include "components/vec3.h"
#include "camera.h"
#include "components/point.h"
#include "components/ray.h"
#include "components/material.h"
#include "components/light.h"
#include "components/illumination.h"
#include "components/intersect_data.h"
#include "world.h"
#include "object.h"
#include "objects/sphere.h"
#include "objects/triangle.h"
#include "textures/checkerboard.h"
#include "textures/noise.h"

// each object is heap allocated & stored in a vector
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
using std::vector;
using std::unique_ptr;
using std::make_unique;

// global constants
const int W = 800;
const int H = 600;
const float FOV_DEG = 90.0f;
const int MAX_DEPTH = 6;

// helper functions
// change of basis into camera space
static inline Point worldToCam(const Point& P, const Point& cam_pos, const Vec3& right, const Vec3& up, const Vec3& forward) {
    Vec3 v(P.getX() - cam_pos.getX(), P.getY() - cam_pos.getY(), P.getZ() - cam_pos.getZ());
    return Point(v.dot(right), v.dot(up), v.dot(forward));
}

static inline Vec3 faceForward(const Vec3& A, const Vec3& B) {
    return (A.dot(B) >= 0.0f) ? A : (A * -1.0f);
}

static inline Color backgroundRadiance(const Vec3& ray_dir) {
    Vec3 dir = ray_dir;
    dir.normalize();
    float t = 0.5f * (dir.getY() + 1.0f);
    Color horizon(18.0f, 24.0f, 30.0f);
    Color zenith(70.0f, 95.0f, 130.0f);
    return (horizon * (1.0f - t)) + (zenith * t);
}

static Color traceRay(
    const Ray& ray,
    int depth,
    const PhongIllumination& phong,
    const std::vector<std::unique_ptr<Light>>& lights,
    const std::vector<std::unique_ptr<Object>>& objects
) {
    Vec3 ray_dir = ray.getDirection();

    if (depth >= MAX_DEPTH) {
        return backgroundRadiance(ray_dir);
    }

    float nearest = std::numeric_limits<float>::infinity();
    Object* obj_hit = nullptr;
    for (const auto& obj : objects) {
        float t;
        if (obj->intersect(ray, t) && t < nearest) {
            nearest = t;
            obj_hit = obj.get();
        }
    }

    if (!obj_hit) {
        return backgroundRadiance(ray_dir);
    }

    Point hit_point(
        ray.getOrigin().getX() + nearest * ray_dir.getX(),
        ray.getOrigin().getY() + nearest * ray_dir.getY(),
        ray.getOrigin().getZ() + nearest * ray_dir.getZ()
    );

    Vec3 normal = obj_hit->normal(hit_point);
    normal.normalize();

    Vec3 view_dir = ray_dir * -1.0f;
    view_dir.normalize();

    IntersectData data;
    data.hit_point = hit_point;
    data.normal = normal;
    data.incoming = ray_dir;
    data.t = nearest;
    data.object = obj_hit;
    data.hit = true;

    const Sphere* sphere = dynamic_cast<const Sphere*>(obj_hit);
    if (sphere) {
        data.uv_coords = sphere->getUV(hit_point);
    }

    Color localColor = phong.computeLocalIllumination(
        data,
        lights,
        objects,
        obj_hit->getMaterial(),
        view_dir,
        obj_hit->getTexture()
    );

    float kr = obj_hit->getMaterial().getReflectivity();
    float kt = obj_hit->getMaterial().getTransmission();
    float ior = obj_hit->getMaterial().getIOR();
    bool insideObject = normal.dot(ray_dir) > 0.0f;
    float localWeight = (kt > 0.0f) ? 0.28f : std::max(0.22f, 1.0f - kr);
    localColor = localColor * localWeight;
    if (depth < MAX_DEPTH) {
        // shared reflection & refraction calculations
        Vec3 I = ray_dir;
        I.normalize();

        float dotIN = I.dot(normal);
        Vec3 reflect_dir = I - (normal * (2.0f * dotIN));
        reflect_dir.normalize();

        const float EPS = 1e-4f;
        auto spawnOffsetPoint = [&](const Vec3& dir) {
            float offsetSign = (dir.dot(normal) >= 0.0f) ? 1.0f : -1.0f;
            return Point(
                hit_point.getX() + normal.getX() * EPS * offsetSign,
                hit_point.getY() + normal.getY() * EPS * offsetSign,
                hit_point.getZ() + normal.getZ() * EPS * offsetSign
            );
        };

        bool tracedReflection = false;
        Color reflectedColor(0, 0, 0);
        auto traceReflection = [&]() {
            if (!tracedReflection) {
                Ray reflectedRay(spawnOffsetPoint(reflect_dir), reflect_dir);
                reflectedColor = traceRay(reflectedRay, depth + 1, phong, lights, objects);
                tracedReflection = true;
            }
            return reflectedColor;
        };

        if (kt > 0.0f) {
            Vec3 orientedNormal = faceForward(normal, I * -1.0f);
            float eta_i = insideObject ? ior : 1.0f;
            float eta_t = insideObject ? 1.0f : ior;
            float eta = eta_i / eta_t;
            float cos_theta_i = -(I.dot(orientedNormal));
            float baseReflectivity = (eta_i - eta_t) / (eta_i + eta_t);
            baseReflectivity = baseReflectivity * baseReflectivity;
            float fresnel = baseReflectivity + (1.0f - baseReflectivity) * std::pow(1.0f - cos_theta_i, 5.0f);
            // t= 𝜂 !𝜂 "i + 𝜂 !𝜂 "cos 𝜃! − 1 − sin 𝜃" # n
            // where: cos 𝜃! = −i . n 
            //        sin 𝜃" # = 𝜂 !𝜂 "#(1 − cos 𝜃! # )
            Vec3 refract_dir = I;
            bool totalInternalReflection = false;

            if (std::fabs(eta_i - eta_t) < 1e-6f) {
                refract_dir.normalize();
            }
            else {
                float sin2_theta_t = eta * eta * (1.0f - cos_theta_i * cos_theta_i);
                if (sin2_theta_t > 1.0f) {
                    totalInternalReflection = true;
                }
                else {
                    float cos_theta_t = std::sqrt(1.0f - sin2_theta_t);
                    Vec3 refract_parallel = I * eta;
                    Vec3 refract_perpendicular = orientedNormal * (eta * cos_theta_i - cos_theta_t);
                    refract_dir = Vec3(
                        refract_parallel.getX() + refract_perpendicular.getX(),
                        refract_parallel.getY() + refract_perpendicular.getY(),
                        refract_parallel.getZ() + refract_perpendicular.getZ()
                    );
                    refract_dir.normalize();
                }
            }

            if (!totalInternalReflection) {
                Ray refractedRay(spawnOffsetPoint(refract_dir), refract_dir);
                Color refractedColor = traceRay(refractedRay, depth + 1, phong, lights, objects);
                localColor = localColor + (refractedColor * kt);
            }

            // for total internal reflection .. rim highlight
            float rimWeight = totalInternalReflection ? 0.10f : (fresnel * 0.04f);
            if (rimWeight > 0.0f) {
                Color rimColor = Color(255, 255, 255) * rimWeight;
                localColor = localColor + rimColor;
            }
        }
        else if (kr > 0.0f) {
            localColor = localColor + (traceReflection() * kr);
        }
    }

    return localColor;
}

// main render function
int renderCPU() {
    const float fov = FOV_DEG * 3.14159265358979323846f / 180.0f;
    const float aspect = (float)W / (float)H;

    // camera from specifications.txt
    Camera camera(Point(0.033089f, 0.765843f, -0.331214f), Vec3(0.033089f, 0.765843f, -1.331214f), Vec3(0.0f, 1.0f, 0.0f), FOV_DEG);
    Point cam_pos = camera.getPosition();
    Point cam_look(0.033089f, 0.765843f, -1.331214f);

    // camera basis in world space
    Vec3 forward(cam_look.getX() - cam_pos.getX(), cam_look.getY() - cam_pos.getY(), cam_look.getZ() - cam_pos.getZ());
    forward.normalize();
    Vec3 world_up(0.0f, 1.0f, 0.0f);
    Vec3 right = forward.cross(world_up);
    right.normalize();
    Vec3 up = right.cross(forward);
    up.normalize();

    // create materials
    Material matGlass = Material::Glass(); // front sphere
    Material matMirror = Material::Mirror(); // back sphere
    Material matRed(Color(80, 10, 10), Color(150, 0, 0), Color(10, 10, 10), 5.0f, 0.0f); // matte red floor

    // create world and add lights
    World world;
    world.setAmbientLight(Color(0, 0, 0)); // no constant ambient term
    
    // light modified from specifications.txt to be more visible in render
    world.addLight(make_unique<PointLight>(
        worldToCam(Point(0.10f, 2.2f, -0.9f), cam_pos, right, up, forward), // light position in camera space
        Color(255, 255, 255), // white light
        0.2f // intensity
    ));

    // create illumination model
    PhongIllumination phong(world.getAmbientLight());

    // create checkerboard texture for floor
    CheckerboardTexture checkerboard(Color(255, 0, 0), Color(255, 255, 0), 1.5f);

    // build scene in world coords
    // sphere #1
    Point s1c_world(0.498855f, 0.393785f, -1.932619f);
    float s1r = 0.36358747f;

    // sphere #2
    Point s2c_world(0.026044f, 0.864156f, -1.366522f);
    float s2r = 0.38744035f;

    // floor as two triangles
    Point floorCenter(1.991213f, -0.257648f, -2.878398f);
    float fx = 6.148293f * 0.5f;
    float fz = 8.200000f * 0.5f;
    float fy = floorCenter.getY();
    Point f00_world(floorCenter.getX() - fx, fy, floorCenter.getZ() - fz);
    Point f10_world(floorCenter.getX() + fx, fy, floorCenter.getZ() - fz);
    Point f01_world(floorCenter.getX() - fx, fy, floorCenter.getZ() + fz);
    Point f11_world(floorCenter.getX() + fx, fy, floorCenter.getZ() + fz);

    // convert scene to camera space
    vector<unique_ptr<Object>> scene_cam;

    // transform centers to camera space
    Point s1c_cam = worldToCam(s1c_world, cam_pos, right, up, forward);
    auto sphere1 = make_unique<Sphere>(s1c_cam, s1r);
    sphere1->setMaterial(matMirror);
    scene_cam.push_back(std::move(sphere1));

    Point s2c_cam = worldToCam(s2c_world, cam_pos, right, up, forward);
    auto sphere2 = make_unique<Sphere>(s2c_cam, s2r);
    sphere2->setMaterial(matGlass);
    scene_cam.push_back(std::move(sphere2));

    // transform vertices to camera space
    Point f00_cam = worldToCam(f00_world, cam_pos, right, up, forward);
    Point f10_cam = worldToCam(f10_world, cam_pos, right, up, forward);
    Point f01_cam = worldToCam(f01_world, cam_pos, right, up, forward);
    Point f11_cam = worldToCam(f11_world, cam_pos, right, up, forward);

    auto t1 = make_unique<Triangle>(f00_cam, f10_cam, f11_cam);
    t1->setMaterial(matRed);
    t1->setTexture(&checkerboard);
    scene_cam.push_back(std::move(t1));
    auto t2 = make_unique<Triangle>(f00_cam, f11_cam, f01_cam);
    t2->setMaterial(matRed);
    t2->setTexture(&checkerboard);
    scene_cam.push_back(std::move(t2));

    // prep image; every pixel will be filled by traced radiance
    Image img(W, H, Color(0, 0, 0));

    // ray trace in camera coords
    float scale = std::tan(fov * 0.5f);
    Point ray_origin(0.0f, 0.0f, 0.0f);

    const int SAMPLES_PER_PIXEL = 4; // 2x2 stratified
    const int SAMPLES_AXIS = 2;

    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            float accumR = 0.0f;
            float accumG = 0.0f;
            float accumB = 0.0f;

            for (int sy = 0; sy < SAMPLES_AXIS; ++sy) {
                for (int sx = 0; sx < SAMPLES_AXIS; ++sx) {
                    float u = (i + (sx + 0.5f) / SAMPLES_AXIS) / (float)W;
                    float v = (j + (sy + 0.5f) / SAMPLES_AXIS) / (float)H;

                    float ndc_x = u * 2.0f - 1.0f;
                    float ndc_y = 1.0f - v * 2.0f;

                    float px = ndc_x * aspect * scale;
                    float py = ndc_y * scale;

                    Vec3 ray_dir(px, py, 1.0f);
                    Ray ray(ray_origin, ray_dir);
                    Color sampleColor = traceRay(ray, 1, phong, world.getLights(), scene_cam);

                    accumR += sampleColor.r;
                    accumG += sampleColor.g;
                    accumB += sampleColor.b;
                }
            }

            float invSamples = 1.0f / (float)SAMPLES_PER_PIXEL;
            Color finalColor(
                accumR * invSamples,
                accumG * invSamples,
                accumB * invSamples
            );
            img.setPixel(i, j, finalColor);
        }
    }

    // std::string out = "output_img.ppm";
    // if (!img.writePPM(out)) return 1;
    // return 0;

    std::string toneMappedOut = "output_img_tonemapped.ppm";
    if (!img.applyToneRepoduction(toneMappedOut)) return 1;
    return 0;
}
