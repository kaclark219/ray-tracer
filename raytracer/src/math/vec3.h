#include <cmath>

class Vec3 {
    private:
        float x, y, z;

    public:
        // default constructor
        Vec3() { x = y = z = 0; }

        // three parameter constructor
        Vec3(float x_val, float y_val, float z_val) {
            x = x_val;
            y = y_val;
            z = z_val;
            this->normalize();
        }

        // copy constructor
        Vec3(const Vec3 &v) : x(v.x), y(v.y), z(v.z) {};

        // getters
        float getX() const { return x; }
        float getY() const { return y; }
        float getZ() const { return z; }

        // setters
        void setX(float x_val) {
            x = x_val;
            this->normalize();
        }
        void setY(float y_val) {
            y = y_val;
            this->normalize();
        }
        void setZ(float z_val) {
            z = z_val;
            this->normalize();
        }

        // normalize the vector
        void normalize() {
            float length = std::sqrt(x*x + y*y + z*z);
            if (length != 0) {
                x /= length;
                y /= length;
                z /= length;
            }
        }
};