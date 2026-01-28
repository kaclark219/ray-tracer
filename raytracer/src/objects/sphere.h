#include "../object.h"
#include "../math/point.h"
#include "../math/vec3.h"

class Sphere : public Object {
    private:
        Vec3 center;
        float radius;

    public:
        // default constructor
        Sphere() : Object(), center(Vec3(0, 0, 0)), radius(1.0f) {}

        // two parameter constructor with optional material index
        Sphere(const Vec3& c, float r, int mat = -1) : Object(mat), center(c), radius(r) {}

        // copy constructor
        Sphere(const Sphere &s) : Object(s), center(s.center), radius(s.radius) {}

        // getters
        Vec3 getCenter() const { return center; }
        float getRadius() const { return radius; }

        // setters
        void setCenter(const Vec3& c) { center = c; }
        void setRadius(float r) { radius = r; }

        // intersection with ray (origin as Point)
        bool intersect(const Point& rayOrigin, const Vec3& rayDirection, float& t) const override {
            Vec3 ro(rayOrigin.getX(), rayOrigin.getY(), rayOrigin.getZ());
            Vec3 oc = ro.subtract(ro, center);
            float a = rayDirection.dot(rayDirection, rayDirection);
            float b = 2.0f * oc.dot(oc, rayDirection);
            float c = oc.dot(oc, oc) - radius * radius;
            float discriminant = b * b - 4 * a * c;
            if (discriminant < 0) {
                return false;
            } else {
                t = (-b - std::sqrt(discriminant)) / (2.0f * a);
                return true;
            }
        }

        Vec3 normal(const Point& p) const override {
            Vec3 vp(p.getX() - center.getX(), p.getY() - center.getY(), p.getZ() - center.getZ());
            return vp; // Vec3 ctor normalizes
        }
};