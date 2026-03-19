#include "image/image.h"
#include "components/vec3.h"
#include "camera.h"
#include "components/point.h"
#include "components/ray.h"
#include "components/material.h"
#include "components/light.h"
#include "components/illumination.h"
#include "components/intersect_data.h"
#include "components/k-d_trees.h"
#include "world.h"
#include "object.h"
#include "objects/sphere.h"
#include "objects/triangle.h"
#include "textures/checkerboard.h"
#include "textures/noise.h"

// each object is heap allocated & stored in a vector
#include <array>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <limits>
using std::vector;
using std::unique_ptr;
using std::make_unique;

// global constants
const int W = 800;
const int H = 600;
const float FOV_DEG = 90.0f;

struct BunnyMesh {
    std::vector<Point> vertices;
    std::vector<std::array<int, 3>> triangles;
    Point minBounds;
    Point maxBounds;
};

struct RenderStats {
    double bruteForceRenderMilliseconds = 0.0;
    double kdTreeBuildMilliseconds = 0.0;
    double kdTreeRenderMilliseconds = 0.0;
};

// helper functions
// change of basis into camera space
static inline Point worldToCam(const Point& P, const Point& cam_pos, const Vec3& right, const Vec3& up, const Vec3& forward) {
    Vec3 v(P.getX() - cam_pos.getX(), P.getY() - cam_pos.getY(), P.getZ() - cam_pos.getZ());
    return Point(v.dot(right), v.dot(up), v.dot(forward));
}

static int parseObjIndex(const std::string& token) {
    const std::size_t slashPos = token.find('/');
    const std::string indexText = slashPos == std::string::npos ? token : token.substr(0, slashPos);
    return std::stoi(indexText) - 1;
}

static BunnyMesh loadBunnyMesh(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open bunny OBJ: " + path);
    }

    BunnyMesh mesh;
    bool hasBounds = false;

    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream stream(line);
        std::string prefix;
        stream >> prefix;

        if (prefix == "v") {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            stream >> x >> y >> z;
            Point vertex(x, y, z);
            mesh.vertices.push_back(vertex);

            if (!hasBounds) {
                mesh.minBounds = vertex;
                mesh.maxBounds = vertex;
                hasBounds = true;
            } else {
                mesh.minBounds.setX(std::min(mesh.minBounds.getX(), x));
                mesh.minBounds.setY(std::min(mesh.minBounds.getY(), y));
                mesh.minBounds.setZ(std::min(mesh.minBounds.getZ(), z));
                mesh.maxBounds.setX(std::max(mesh.maxBounds.getX(), x));
                mesh.maxBounds.setY(std::max(mesh.maxBounds.getY(), y));
                mesh.maxBounds.setZ(std::max(mesh.maxBounds.getZ(), z));
            }
        } else if (prefix == "f") {
            std::string a;
            std::string b;
            std::string c;
            stream >> a >> b >> c;
            if (!a.empty() && !b.empty() && !c.empty()) {
                mesh.triangles.push_back({parseObjIndex(a), parseObjIndex(b), parseObjIndex(c)});
            }
        }
    }

    if (mesh.vertices.empty() || mesh.triangles.empty()) {
        throw std::runtime_error("Bunny OBJ did not contain vertices and triangles: " + path);
    }

    return mesh;
}

static Point normalizeBunnyVertex(const Point& vertex, const BunnyMesh& mesh) {
    const float centerX = 0.5f * (mesh.minBounds.getX() + mesh.maxBounds.getX());
    const float baseY = mesh.minBounds.getY();
    const float centerZ = 0.5f * (mesh.minBounds.getZ() + mesh.maxBounds.getZ());

    return Point(
        vertex.getX() - centerX,
        vertex.getY() - baseY,
        vertex.getZ() - centerZ
    );
}

static void addBunnyInstance(
    std::vector<std::unique_ptr<Object>>& scene_cam,
    const BunnyMesh& mesh,
    const Point& worldPosition,
    float targetHeight,
    const Material& material,
    const Point& cam_pos,
    const Vec3& right,
    const Vec3& up,
    const Vec3& forward
) {
    const float sourceHeight = mesh.maxBounds.getY() - mesh.minBounds.getY();
    const float scale = sourceHeight > 0.0f ? targetHeight / sourceHeight : 1.0f;

    for (const auto& tri : mesh.triangles) {
        const Point p0 = normalizeBunnyVertex(mesh.vertices[tri[0]], mesh);
        const Point p1 = normalizeBunnyVertex(mesh.vertices[tri[1]], mesh);
        const Point p2 = normalizeBunnyVertex(mesh.vertices[tri[2]], mesh);

        const Point w0(
            worldPosition.getX() + p0.getX() * scale,
            worldPosition.getY() + p0.getY() * scale,
            worldPosition.getZ() + p0.getZ() * scale
        );
        const Point w1(
            worldPosition.getX() + p1.getX() * scale,
            worldPosition.getY() + p1.getY() * scale,
            worldPosition.getZ() + p1.getZ() * scale
        );
        const Point w2(
            worldPosition.getX() + p2.getX() * scale,
            worldPosition.getY() + p2.getY() * scale,
            worldPosition.getZ() + p2.getZ() * scale
        );

        auto triangle = make_unique<Triangle>(
            worldToCam(w0, cam_pos, right, up, forward),
            worldToCam(w1, cam_pos, right, up, forward),
            worldToCam(w2, cam_pos, right, up, forward)
        );
        triangle->setMaterial(material);
        scene_cam.push_back(std::move(triangle));
    }
}

static bool findNearestLinear(
    const Ray& ray,
    const std::vector<std::unique_ptr<Object>>& scene,
    float& nearest,
    Object*& objHit,
    float minT = 1e-6f
) {
    nearest = std::numeric_limits<float>::infinity();
    objHit = nullptr;

    for (const auto& obj : scene) {
        float t = 0.0f;
        if (obj->intersect(ray, t) && t > minT && t < nearest) {
            nearest = t;
            objHit = obj.get();
        }
    }

    return objHit != nullptr;
}

static bool findNearestKDTree(
    const Ray& ray,
    const KDTree& tree,
    float& nearest,
    Object*& objHit
) {
    KDTree::HitResult result;
    if (!tree.intersect(ray, result)) {
        nearest = std::numeric_limits<float>::infinity();
        objHit = nullptr;
        return false;
    }

    nearest = result.t;
    objHit = const_cast<Object*>(result.object);
    return objHit != nullptr;
}

static Color shadePoint(
    const IntersectData& data,
    const std::vector<std::unique_ptr<Light>>& lights,
    const std::vector<std::unique_ptr<Object>>& scene,
    const Color& ambientLight,
    const Vec3& viewDir,
    bool useKDTree,
    const KDTree* tree
) {
    Color diffuseColor = data.object->getMaterial().getDiffuse();
    if (data.object->getTexture() != nullptr) {
        diffuseColor = data.object->getTexture()->sample(
            data.hit_point,
            data.uv_coords,
            data.normal
        );
    }

    Color result = data.object->getMaterial().getAmbient() * ambientLight;

    for (const auto& light : lights) {
        Vec3 L = light->getPosition() - data.hit_point;
        const float lightDist = L.length();
        L.normalize();

        Vec3 N = data.normal;
        if (N.dot(viewDir) < 0.0f) {
            N = N * -1.0f;
        }

        const float EPS = 1e-3f;
        Point shadowOrigin(
            data.hit_point.getX() + N.getX() * EPS,
            data.hit_point.getY() + N.getY() * EPS,
            data.hit_point.getZ() + N.getZ() * EPS
        );
        Ray shadowRay(shadowOrigin, L);

        bool inShadow = false;
        if (useKDTree && tree != nullptr) {
            inShadow = tree->occluded(shadowRay, lightDist);
        } else {
            float tShadow = 0.0f;
            Object* blocker = nullptr;
            if (findNearestLinear(shadowRay, scene, tShadow, blocker, EPS) && tShadow < lightDist) {
                inShadow = true;
            }
        }

        if (inShadow) {
            continue;
        }

        const float NdotL = std::max(N.dot(L), 0.0f);
        Vec3 R = (N * (2.0f * NdotL)) - L;
        R.normalize();
        const float RdotV = std::max(R.dot(viewDir), 0.0f);
        const float specularFactor = std::pow(RdotV, data.object->getMaterial().getShininess());
        const Color lightColor = light->getColor() * light->getIntensity();

        result = result + (diffuseColor * lightColor * NdotL);
        result = result + (data.object->getMaterial().getSpecular() * lightColor * specularFactor);
    }

    result.clamp();
    return result;
}

static double renderSceneToImage(
    Image& img,
    const std::vector<std::unique_ptr<Object>>& scene,
    const World& world,
    bool useKDTree,
    const KDTree* tree,
    float fov,
    float aspect
) {
    const auto start = std::chrono::high_resolution_clock::now();

    // ray trace in camera coords
    const float scale = std::tan(fov * 0.5f);
    const Point ray_origin(0.0f, 0.0f, 0.0f);

    const int SAMPLES_PER_PIXEL = 4; // 2x2 stratified
    const int SAMPLES_AXIS = 2;

    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            float accumR = 0.0f;
            float accumG = 0.0f;
            float accumB = 0.0f;

            for (int sy = 0; sy < SAMPLES_AXIS; ++sy) {
                for (int sx = 0; sx < SAMPLES_AXIS; ++sx) {
                    const float u = (i + (sx + 0.5f) / SAMPLES_AXIS) / (float)W;
                    const float v = (j + (sy + 0.5f) / SAMPLES_AXIS) / (float)H;

                    const float ndc_x = u * 2.0f - 1.0f;
                    const float ndc_y = 1.0f - v * 2.0f;

                    const float px = ndc_x * aspect * scale;
                    const float py = ndc_y * scale;

                    Vec3 ray_dir(px, py, 1.0f);
                    Ray ray(ray_origin, ray_dir);

                    float nearest = std::numeric_limits<float>::infinity();
                    Object* obj_hit = nullptr;
                    const bool hit = useKDTree && tree != nullptr
                        ? findNearestKDTree(ray, *tree, nearest, obj_hit)
                        : findNearestLinear(ray, scene, nearest, obj_hit);

                    Color sampleColor(135, 206, 235);
                    if (hit && obj_hit != nullptr) {
                        Point hit_point(
                            ray_origin.getX() + nearest * ray_dir.getX(),
                            ray_origin.getY() + nearest * ray_dir.getY(),
                            ray_origin.getZ() + nearest * ray_dir.getZ()
                        );
                        Vec3 normal = obj_hit->normal(hit_point);
                        normal.normalize();

                        Vec3 view_dir = ray_dir * -1.0f;
                        view_dir.normalize();

                        if (normal.dot(view_dir) < 0.0f) {
                            normal = normal * -1.0f;
                        }

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

                        sampleColor = shadePoint(
                            data,
                            world.getLights(),
                            scene,
                            world.getAmbientLight(),
                            view_dir,
                            useKDTree,
                            tree
                        );
                    }

                    accumR += sampleColor.r;
                    accumG += sampleColor.g;
                    accumB += sampleColor.b;
                }
            }

            const float invSamples = 1.0f / (float)SAMPLES_PER_PIXEL;
            Color finalColor(
                (int)(accumR * invSamples),
                (int)(accumG * invSamples),
                (int)(accumB * invSamples)
            );
            finalColor.clamp();
            img.setPixel(i, j, finalColor);
        }
    }

    const auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
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
    Material matWhiteGrey(Color(55, 55, 55), Color(230, 230, 230), Color(200, 200, 200), 60.0f, 0.0f); // white/grey bunny
    Material matGrey(Color(40, 40, 40), Color(190, 190, 190), Color(130, 130, 130), 50.0f, 0.0f); // shiny grey sphere
    Material matRed(Color(20, 0, 0), Color(150, 0, 0), Color(10, 10, 10), 5.0f, 0.0f); // matte red floor

    // create world and add lights
    World world;
    world.setAmbientLight(Color(40, 40, 40)); // ambient light
    
    // light modified from specifications.txt to be more visible in render
    world.addLight(make_unique<PointLight>(
        worldToCam(Point(0.262f, 2.8f, -1.2f), cam_pos, right, up, forward), // light position in camera space
        Color(255, 255, 255), // white light
        1.8f // intensity
    ));
    world.addLight(make_unique<PointLight>(
        worldToCam(Point(-1.2f, 1.8f, -1.0f), cam_pos, right, up, forward), // fill light in camera space
        Color(255, 255, 255), // white light
        1.2f // intensity
    ));

    // create checkerboard texture for floor
    CheckerboardTexture checkerboard(Color(255, 0, 0), Color(255, 255, 0), 1.5f);

    // create perlin noise texture for floor
    // NoiseTexture noiseTexture(2.0f, Color(0, 0, 0), Color(255, 255, 255));

    // build scene in world coords
    // floor as two triangles
    Point floorCenter(1.991213f, -0.257648f, -2.878398f);
    float fx = 6.148293f * 0.5f;
    float fz = 5.984314f * 0.5f;
    float fy = floorCenter.getY();
    Point f00_world(floorCenter.getX() - fx, fy, floorCenter.getZ() - fz);
    Point f10_world(floorCenter.getX() + fx, fy, floorCenter.getZ() - fz);
    Point f01_world(floorCenter.getX() - fx, fy, floorCenter.getZ() + fz);
    Point f11_world(floorCenter.getX() + fx, fy, floorCenter.getZ() + fz);

    // convert scene to camera space
    vector<unique_ptr<Object>> scene_cam;

    const BunnyMesh bunnyMesh = loadBunnyMesh("src/objects/bunny.obj");
    const Point bunnyBaseWorld(cam_look.getX(), fy, -1.95f);
    const float bunnyHeight = 1.35f;
    addBunnyInstance(
        scene_cam,
        bunnyMesh,
        bunnyBaseWorld,
        bunnyHeight,
        matWhiteGrey,
        cam_pos,
        right,
        up,
        forward
    );

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

    RenderStats stats;

    Image bruteForceImg(W, H, Color(135, 206, 235));
    stats.bruteForceRenderMilliseconds = renderSceneToImage(
        bruteForceImg,
        scene_cam,
        world,
        false,
        nullptr,
        fov,
        aspect
    );

    KDTree kdTree;
    kdTree.build(scene_cam);
    stats.kdTreeBuildMilliseconds = kdTree.getBuildMilliseconds();

    Image img(W, H, Color(135, 206, 235));
    stats.kdTreeRenderMilliseconds = renderSceneToImage(
        img,
        scene_cam,
        world,
        true,
        &kdTree,
        fov,
        aspect
    );

    std::string bruteForceOut = "output_img_no_kdtree.ppm";
    if (!bruteForceImg.writePPM(bruteForceOut)) return 1;

    std::string kdTreeOut = "output_img_kdtree.ppm";
    if (!img.writePPM(kdTreeOut)) return 1;

    std::cout << "KD-tree build time: " << stats.kdTreeBuildMilliseconds << " ms\n";
    std::cout << "Render time without KD-tree: " << stats.bruteForceRenderMilliseconds << " ms\n";
    std::cout << "Render time with KD-tree: " << stats.kdTreeRenderMilliseconds << " ms\n";
    std::cout << "KD-tree nodes: " << kdTree.getNodeCount() << "\n";
    std::cout << "KD-tree leaves: " << kdTree.getLeafCount() << "\n";
    std::cout << "Unsupported primitives skipped by KD-tree: " << kdTree.getUnsupportedPrimitiveCount() << "\n";
    return 0;
}
