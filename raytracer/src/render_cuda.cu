#include <cuda_runtime.h>
#include "image/image.h"

#include "components/material.h"
#include "components/illumination.h"
#include "objects/sphere.h"
#include "objects/triangle.h"
#include "components/color.h"
#include "components/point.h"
#include "components/vec3.h"
#include "components/ray.h"
#include "components/light.h"
#include "components/intersect_data.h"
#include "textures/checkerboard.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>

// global constants
const int W = 800;
const int H = 600;
const float FOV_DEG = 90.0f;
const int MAX_DEPTH = 6;

// cuda error checking
static inline void cudaCheck(cudaError_t err, const char* file, int line) {
    if (err != cudaSuccess) {
        std::fprintf(stderr, "CUDA error %s:%d: %s\n", file, line, cudaGetErrorString(err));
        std::exit(1);
    }
}
#define CUDA_CHECK(x) cudaCheck((x), __FILE__, __LINE__)

#ifdef __CUDACC__
    #define CUDA_DEVICE __device__
#else
    #define CUDA_DEVICE
#endif

// helper functions
// change of basis from world to camera space
static inline Point worldToCam(const Point& P, const Point& cam_pos, const Vec3& right, const Vec3& up, const Vec3& forward) {
    Vec3 v(P.getX() - cam_pos.getX(), P.getY() - cam_pos.getY(), P.getZ() - cam_pos.getZ());
    return Point(v.dot(right), v.dot(up), v.dot(forward));
}

CUDA_DEVICE Vec3 faceForwardGPU(const Vec3& A, const Vec3& B) {
    return (A.dot(B) >= 0.0f) ? A : (A * -1.0f);
}

CUDA_DEVICE Color traceRayGPU(
    const Ray& ray,
    int depth,
    const SphereGPU* spheres,
    int nSpheres,
    const TriangleGPU* tris,
    int nTris,
    const Material* materials,
    int numMaterials,
    const LightData* lights,
    int numLights,
    const CheckerboardTextureData* floorTexture,
    const Color& ambientLight,
    const Color& background
) {
    if (depth >= MAX_DEPTH) {
        return background;
    }

    float nearest = 1e30f;
    int hitType = -1; // 0 = sphere, 1 = triangle
    int hitIndex = -1;

    for (int s = 0; s < nSpheres; ++s) {
        float t;
        if (intersectSphereGPU(spheres[s], ray, t) && t < nearest) {
            nearest = t;
            hitType = 0;
            hitIndex = s;
        }
    }
    for (int tIdx = 0; tIdx < nTris; ++tIdx) {
        float t;
        if (intersectTriangleGPU(tris[tIdx], ray, t) && t < nearest) {
            nearest = t;
            hitType = 1;
            hitIndex = tIdx;
        }
    }

    if (hitType == -1) {
        return background;
    }

    Vec3 ray_dir = ray.getDirection();
    Point hit_point(
        ray.getOrigin().getX() + nearest * ray_dir.getX(),
        ray.getOrigin().getY() + nearest * ray_dir.getY(),
        ray.getOrigin().getZ() + nearest * ray_dir.getZ()
    );

    Vec3 normal;
    int matIndex = 0;
    if (hitType == 0) {
        normal = Vec3(
            hit_point.getX() - spheres[hitIndex].center.getX(),
            hit_point.getY() - spheres[hitIndex].center.getY(),
            hit_point.getZ() - spheres[hitIndex].center.getZ()
        );
        matIndex = spheres[hitIndex].materialIndex;
    } else {
        Vec3 edge1 = Vec3(
            tris[hitIndex].points[1].getX() - tris[hitIndex].points[0].getX(),
            tris[hitIndex].points[1].getY() - tris[hitIndex].points[0].getY(),
            tris[hitIndex].points[1].getZ() - tris[hitIndex].points[0].getZ()
        );
        Vec3 edge2 = Vec3(
            tris[hitIndex].points[2].getX() - tris[hitIndex].points[0].getX(),
            tris[hitIndex].points[2].getY() - tris[hitIndex].points[0].getY(),
            tris[hitIndex].points[2].getZ() - tris[hitIndex].points[0].getZ()
        );
        normal = edge1.cross(edge2);
        matIndex = tris[hitIndex].materialIndex;
    }
    normal.normalize();
    if (matIndex < 0 || matIndex >= numMaterials) {
        matIndex = 0;
    }

    Vec3 view_dir = ray_dir * -1.0f;
    view_dir.normalize();

    Color localColor = materials[matIndex].getAmbient() * ambientLight;
    const float EPS = 1e-4f;
    for (int li = 0; li < numLights; ++li) {
        Vec3 L = lights[li].position - hit_point;
        float lightDist = L.length();
        L.normalize();

        float NdotL = normal.dot(L);
        if (NdotL < 0.0f) NdotL = 0.0f;

        Point shadow_origin(
            hit_point.getX() + normal.getX() * EPS,
            hit_point.getY() + normal.getY() * EPS,
            hit_point.getZ() + normal.getZ() * EPS
        );
        Ray shadow_ray(shadow_origin, L);
        bool inShadow = false;
        float shadowTransmission = 1.0f;
        for (int s = 0; s < nSpheres; ++s) {
            float tShadow;
            if (intersectSphereGPU(spheres[s], shadow_ray, tShadow) && tShadow > EPS && tShadow < lightDist) {
                float ktShadow = materials[spheres[s].materialIndex].getTransmission();
                if (ktShadow <= 0.0f) {
                    inShadow = true;
                    break;
                }
                shadowTransmission *= (1.0f - (0.35f * ktShadow));
            }
        }
        if (!inShadow) {
            for (int tIdx = 0; tIdx < nTris; ++tIdx) {
                float tShadow;
                if (intersectTriangleGPU(tris[tIdx], shadow_ray, tShadow) && tShadow > EPS && tShadow < lightDist) {
                    float ktShadow = materials[tris[tIdx].materialIndex].getTransmission();
                    if (ktShadow <= 0.0f) {
                        inShadow = true;
                        break;
                    }
                    shadowTransmission *= (1.0f - (0.35f * ktShadow));
                }
            }
        }
        if (inShadow) {
            Color lightColor = lights[li].color * lights[li].intensity;
            Color diffuseColor = materials[matIndex].getDiffuse();
            if (matIndex == 2 && floorTexture != nullptr) {
                diffuseColor = sampleCheckerboardGPU(*floorTexture, hit_point);
            }
            const float SHADOW_BOUNCE = 0.30f;
            localColor = localColor + (diffuseColor * lightColor * NdotL * SHADOW_BOUNCE);
            continue;
        }

        Vec3 R = (normal * (2.0f * NdotL)) - L;
        R.normalize();
        float RdotV = R.dot(view_dir);
        if (RdotV < 0.0f) RdotV = 0.0f;
        float specularFactor = (RdotV > 0.0f) ? powf(RdotV, materials[matIndex].getShininess()) : 0.0f;

        Color lightColor = lights[li].color * (lights[li].intensity * shadowTransmission);
        Color diffuseColor = materials[matIndex].getDiffuse();
        if (matIndex == 2 && floorTexture != nullptr) {
            diffuseColor = sampleCheckerboardGPU(*floorTexture, hit_point);
        }

        localColor = localColor + (diffuseColor * lightColor * NdotL);
        localColor = localColor + (materials[matIndex].getSpecular() * lightColor * specularFactor);
    }

    float kr = materials[matIndex].getReflectivity();
    float kt = materials[matIndex].getTransmission();
    float ior = materials[matIndex].getIOR();
    bool insideObject = normal.dot(ray_dir) > 0.0f;
    float localWeight = (kt > 0.0f) ? 0.28f : fmaxf(0.22f, 1.0f - kr);
    localColor = localColor * localWeight;

    Vec3 I = ray_dir;
    I.normalize();
    float dotIN = I.dot(normal);
    Vec3 reflect_dir = I - (normal * (2.0f * dotIN));
    reflect_dir.normalize();

    auto spawnOffsetPoint = [&](const Vec3& dir) {
        float offsetSign = (dir.dot(normal) >= 0.0f) ? 1.0f : -1.0f;
        return Point(
            hit_point.getX() + normal.getX() * EPS * offsetSign,
            hit_point.getY() + normal.getY() * EPS * offsetSign,
            hit_point.getZ() + normal.getZ() * EPS * offsetSign
        );
    };

    if (kt > 0.0f) {
        Vec3 orientedNormal = faceForwardGPU(normal, I * -1.0f);
        float eta_i = insideObject ? ior : 1.0f;
        float eta_t = insideObject ? 1.0f : ior;
        float eta = eta_i / eta_t;
        float cos_theta_i = -(I.dot(orientedNormal));
        float baseReflectivity = (eta_i - eta_t) / (eta_i + eta_t);
        baseReflectivity = baseReflectivity * baseReflectivity;
        float fresnel = baseReflectivity + (1.0f - baseReflectivity) * powf(1.0f - cos_theta_i, 5.0f);
        Vec3 refract_dir = I;
        bool totalInternalReflection = false;

        if (fabsf(eta_i - eta_t) < 1e-6f) {
            refract_dir.normalize();
        } else {
            float sin2_theta_t = eta * eta * (1.0f - cos_theta_i * cos_theta_i);
            if (sin2_theta_t > 1.0f) {
                totalInternalReflection = true;
            } else {
                float cos_theta_t = sqrtf(1.0f - sin2_theta_t);
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
            Color refractedColor = traceRayGPU(
                refractedRay, depth + 1,
                spheres, nSpheres, tris, nTris, materials, numMaterials,
                lights, numLights, floorTexture, ambientLight, background
            );
            localColor = localColor + (refractedColor * kt);
        }

        float rimWeight = totalInternalReflection ? 0.10f : (fresnel * 0.04f);
        if (rimWeight > 0.0f) {
            localColor = localColor + (Color(255, 255, 255) * rimWeight);
        }
    } else if (kr > 0.0f) {
        Ray reflectedRay(spawnOffsetPoint(reflect_dir), reflect_dir);
        Color reflectedColor = traceRayGPU(
            reflectedRay, depth + 1,
            spheres, nSpheres, tris, nTris, materials, numMaterials,
            lights, numLights, floorTexture, ambientLight, background
        );
        localColor = localColor + (reflectedColor * kr);
    }

    localColor.clamp();
    return localColor;
}

// one thread per pixel
__global__ void renderKernel(Color* fb, int w, int h, float aspect, float scale,
    const SphereGPU* spheres, int nSpheres,
    const TriangleGPU* tris, int nTris,
    const Material* materials, int numMaterials,
    const LightData* lights, int numLights,
    const CheckerboardTextureData* floorTexture,
    Color ambientLight, Color background) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    if (i >= w || j >= h) return;

    float ndc_x = ((i + 0.5f) / (float)w) * 2.0f - 1.0f;
    float ndc_y = 1.0f - ((j + 0.5f) / (float)h) * 2.0f;

    float px = ndc_x * aspect * scale;
    float py = ndc_y * scale;

    Point ray_origin(0.0f, 0.0f, 0.0f);
    Vec3 ray_dir(px, py, 1.0f);
    Ray ray(ray_origin, ray_dir);

    Color result = traceRayGPU(
        ray, 1,
        spheres, nSpheres,
        tris, nTris,
        materials, numMaterials,
        lights, numLights,
        floorTexture,
        ambientLight,
        background
    );

    fb[(size_t)j * (size_t)w + (size_t)i] = result;
}

// main render function
int renderCUDA() {
    const float PI = 3.14159265358979323846f;
    const float fov = FOV_DEG * PI / 180.0f;
    const float aspect = (float)W / (float)H;
    const float scale = tanf(fov * 0.5f);

    // camera values from specifications.txt
    Point cam_pos(0.033089f, 0.765843f, -0.331214f);
    Point cam_look(0.033089f, 0.765843f, -1.331214f);

    // camera basis in world space
    Vec3 forward(cam_look.getX() - cam_pos.getX(), cam_look.getY() - cam_pos.getY(), cam_look.getZ() - cam_pos.getZ());
    forward.normalize();

    Vec3 world_up(0.0f, 1.0f, 0.0f);

    Vec3 right = forward.cross(world_up);
    right.normalize();

    Vec3 up = right.cross(forward);
    up.normalize();

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
    Point s1c_cam = worldToCam(s1c_world, cam_pos, right, up, forward);
    Point s2c_cam = worldToCam(s2c_world, cam_pos, right, up, forward);

    Point f00_cam = worldToCam(f00_world, cam_pos, right, up, forward);
    Point f10_cam = worldToCam(f10_world, cam_pos, right, up, forward);
    Point f01_cam = worldToCam(f01_world, cam_pos, right, up, forward);
    Point f11_cam = worldToCam(f11_world, cam_pos, right, up, forward);

    // materials
    Material hMats[3];
    hMats[0] = Material::Mirror();
    hMats[1] = Material::Glass();
    hMats[2] = Material(Color(80, 10, 10), Color(150, 0, 0), Color(10, 10, 10), 5.0f, 0.0f);

    // lights
    LightData hLights[1];
    hLights[0] = LightData(
        worldToCam(Point(0.10f, 2.2f, -0.9f), cam_pos, right, up, forward),
        Color(255, 255, 255),
        0.9f
    );
    Color ambientLight(70, 70, 70);

    // create checkerboard texture for floor
    CheckerboardTextureData hCheckboard(Color(255, 0, 0), Color(255, 255, 0), 1.5f);

    // host gpu-scene arrays
    SphereGPU hSpheres[2];
    hSpheres[0].center = s1c_cam;
    hSpheres[0].radius = s1r;
    hSpheres[0].materialIndex = 0;
    hSpheres[0].color = Color(255, 255, 255); // unused in shading

    hSpheres[1].center = s2c_cam;
    hSpheres[1].radius = s2r;
    hSpheres[1].materialIndex = 1;
    hSpheres[1].color = Color(255, 255, 255); // unused in shading

    TriangleGPU hTris[2];
    hTris[0].points[0] = f00_cam;
    hTris[0].points[1] = f10_cam;
    hTris[0].points[2] = f11_cam;
    hTris[0].materialIndex = 2;
    hTris[0].color = Color(255, 0, 0); // unused in shading

    hTris[1].points[0] = f00_cam;
    hTris[1].points[1] = f11_cam;
    hTris[1].points[2] = f01_cam;
    hTris[1].materialIndex = 2;
    hTris[1].color = Color(255, 0, 0); // unused in shading

    // allocate device memory
    Color* dFB = nullptr;
    SphereGPU* dSpheres = nullptr;
    TriangleGPU* dTris = nullptr;
    Material* dMats = nullptr;
    LightData* dLights = nullptr;
    CheckerboardTextureData* dCheckboard = nullptr;

    size_t fbBytes = (size_t)W * (size_t)H * sizeof(Color);

    CUDA_CHECK(cudaMalloc(&dFB, fbBytes));
    CUDA_CHECK(cudaMalloc(&dSpheres, 2 * sizeof(SphereGPU)));
    CUDA_CHECK(cudaMalloc(&dTris, 2 * sizeof(TriangleGPU)));
    CUDA_CHECK(cudaMalloc(&dMats, 3 * sizeof(Material)));
    CUDA_CHECK(cudaMalloc(&dLights, 1 * sizeof(LightData)));
    CUDA_CHECK(cudaMalloc(&dCheckboard, sizeof(CheckerboardTextureData)));

    CUDA_CHECK(cudaMemcpy(dSpheres, hSpheres, 2 * sizeof(SphereGPU), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(dTris, hTris, 2 * sizeof(TriangleGPU), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(dMats, hMats, 3 * sizeof(Material), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(dLights, hLights, 1 * sizeof(LightData), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(dCheckboard, &hCheckboard, sizeof(CheckerboardTextureData), cudaMemcpyHostToDevice));

    // launch kernel
    dim3 block(16, 16);
    dim3 grid((W + block.x - 1) / block.x, (H + block.y - 1) / block.y);

    Color bg(135, 206, 235); // sky blue background
    renderKernel<<<grid, block>>>(dFB, W, H, aspect, scale,
        dSpheres, 2, dTris, 2,
        dMats, 3,
        dLights, 1,
        dCheckboard,
        ambientLight, bg);

    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaDeviceSynchronize());

    // copy framebuffer back
    std::vector<Color> hFB((size_t)W * (size_t)H);
    CUDA_CHECK(cudaMemcpy(hFB.data(), dFB, fbBytes, cudaMemcpyDeviceToHost));

    // cleanup device memory
    CUDA_CHECK(cudaFree(dFB));
    CUDA_CHECK(cudaFree(dSpheres));
    CUDA_CHECK(cudaFree(dTris));
    CUDA_CHECK(cudaFree(dMats));
    CUDA_CHECK(cudaFree(dLights));
    CUDA_CHECK(cudaFree(dCheckboard));

    // write to image class
    Image img(W, H, bg);
    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            img.setPixel(i, j, hFB[(size_t)j * (size_t)W + (size_t)i]);
        }
    }

    std::string out = "output_img_gpu.ppm";
    if (!img.writePPM(out)) return 1;

    return 0;
}

#undef CUDA_DEVICE
