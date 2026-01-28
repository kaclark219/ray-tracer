#include "math/point.h"
#include "math/vec3.h"

class Object {
    protected:
        int materialIndex;
    public:
        Object() : materialIndex(-1) {}
        explicit Object(int matIndex) : materialIndex(matIndex) {}
        virtual ~Object() = default;

        int getMaterialIndex() const { return materialIndex; }
        void setMaterialIndex(int matIndex) { materialIndex = matIndex; }

        // intersect ray (origin as Point, direction as Vec3), returns hit distance in t
        virtual bool intersect(const Point& rayOrigin, const Vec3& rayDirection, float& t) const = 0;

        // surface normal (unit) at surface point p
        virtual Vec3 normal(const Point& p) const = 0;
};