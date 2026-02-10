#include "image/image.h"
#include "components/vec3.h"
#include "camera.h"
#include "components/point.h"
#include "components/ray.h"
#include "object.h"
#include "objects/sphere.h"
#include "objects/triangle.h"

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

// helper functions
// change of basis into camera space
static inline Point worldToCam(const Point& P, const Point& cam_pos, const Vec3& right, const Vec3& up, const Vec3& forward) {
    Vec3 v(P.getX() - cam_pos.getX(), P.getY() - cam_pos.getY(), P.getZ() - cam_pos.getZ());
    return Point(v.dot(right), v.dot(up), v.dot(forward));
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
    float fz = 5.984314f * 0.5f;
    float fy = floorCenter.getY();
    Point f00_world(floorCenter.getX() - fx, fy, floorCenter.getZ() - fz);
    Point f10_world(floorCenter.getX() + fx, fy, floorCenter.getZ() - fz);
    Point f01_world(floorCenter.getX() - fx, fy, floorCenter.getZ() + fz);
    Point f11_world(floorCenter.getX() + fx, fy, floorCenter.getZ() + fz);

    // convert scene to camera space
    vector<unique_ptr<Object>> scene_cam;

    // transform centers to camera space
    Point s1c_cam = worldToCam(s1c_world, cam_pos, right, up, forward);
    scene_cam.push_back(make_unique<Sphere>(s1c_cam, s1r, -1, Color(255, 255, 0))); // yellow sphere

    Point s2c_cam = worldToCam(s2c_world, cam_pos, right, up, forward);
    scene_cam.push_back(make_unique<Sphere>(s2c_cam, s2r, -1, Color(200, 200, 200))); // grey sphere

    // transform vertices to camera space
    Point f00_cam = worldToCam(f00_world, cam_pos, right, up, forward);
    Point f10_cam = worldToCam(f10_world, cam_pos, right, up, forward);
    Point f01_cam = worldToCam(f01_world, cam_pos, right, up, forward);
    Point f11_cam = worldToCam(f11_world, cam_pos, right, up, forward);

    auto t1 = make_unique<Triangle>(f00_cam, f10_cam, f11_cam, -1);
    t1->setColor(Color(255, 0, 0)); // red
    scene_cam.push_back(std::move(t1));
    auto t2 = make_unique<Triangle>(f00_cam, f11_cam, f01_cam, -1);
    t2->setColor(Color(255, 0, 0)); // red
    scene_cam.push_back(std::move(t2));

    // prep img with sky blue background
    Image img(W, H, Color(135, 206, 235));

    // ray trace in camera coords
    float scale = std::tan(fov * 0.5f);
    Point ray_origin(0.0f, 0.0f, 0.0f);

    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            float ndc_x = ((i + 0.5f) / W) * 2.0f - 1.0f;
            float ndc_y = 1.0f - ((j + 0.5f) / H) * 2.0f;

            float px = ndc_x * aspect * scale;
            float py = ndc_y * scale;

            Vec3 ray_dir(px, py, 1.0f);
            Ray ray(ray_origin, ray_dir);

            float nearest = std::numeric_limits<float>::infinity();
            Object* obj_hit = nullptr;

            for (const auto& obj : scene_cam) {
                float t;
                if (obj->intersect(ray, t) && t < nearest) {
                    nearest = t;
                    obj_hit = obj.get();
                }
            }

            if (obj_hit)
                img.setPixel(i, j, obj_hit->getColor());
        }
    }

    std::string out = "output_img.ppm";
    if (!img.writePPM(out)) return 1;
    return 0;
}