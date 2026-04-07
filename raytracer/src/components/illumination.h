#ifndef ILLUMINATION_H
#define ILLUMINATION_H

#include "intersect_data.h"
#include "light.h"
#include "material.h"
#include "object.h"
#include "../textures/texture.h"
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

#ifdef __CUDACC__
    #define CUDA_CALLABLE __host__ __device__
#else
    #define CUDA_CALLABLE
#endif

// FOR CPU RENDERING
class Illumination {
    public:
        virtual ~Illumination() = default;

        // virtual method to compute illumination
        virtual Color illuminate(const IntersectData& data, const std::vector<std::unique_ptr<Light>>& lights,
            const std::vector<std::unique_ptr<Object>>& objects,
            const Material& material, const Vec3& view_dir, const Texture* texture = nullptr) const = 0;
};

class PhongIllumination : public Illumination {
    private:
        Color ambientLight; // global ambient light color/intensity

        Color combineWithReflection(const Color& localColor, const Material& material, const Color& reflectedColor) const {
            Color result = localColor + (reflectedColor * material.getReflectivity());
            result.clamp();
            return result;
        }
    
    public:
        // constructor with ambient light
        PhongIllumination(const Color& ambient = Color(50, 50, 50)) : ambientLight(ambient) {}

        Color computeLocalIllumination(const IntersectData& data, const std::vector<std::unique_ptr<Light>>& lights,
            const std::vector<std::unique_ptr<Object>>& objects,
            const Material& material, const Vec3& view_dir, const Texture* texture = nullptr) const {
            
            // sample texture if available & get diffuse color
            Color diffuseColor = material.getDiffuse();
            if (texture != nullptr) {
                Color textureSample = texture->sample(data.hit_point, data.uv_coords, data.normal);
                // use texture color directly
                diffuseColor = textureSample;
            }
            
            Color result = material.getAmbient() * ambientLight;
            // compute diffuse and specular contributions from each light
            for (const auto& light : lights) {
                Vec3 L = light->getPosition() - data.hit_point;
                float lightDist = L.length();
                L.normalize();
                Vec3 N = data.normal;
                float NdotL = std::max(N.dot(L), 0.0f);

                // shadow ray: check if light is blocked
                const float EPS = 1e-4f;
                Point shadow_origin(
                    data.hit_point.getX() + N.getX() * EPS,
                    data.hit_point.getY() + N.getY() * EPS,
                    data.hit_point.getZ() + N.getZ() * EPS
                );
                Ray shadow_ray(shadow_origin, L);
                bool inShadow = false;
                for (const auto& obj : objects) {
                    float tShadow;
                    if (obj->intersect(shadow_ray, tShadow) && tShadow > EPS && tShadow < lightDist) {
                        inShadow = true;
                        break;
                    }
                }
                if (inShadow) {
                    Color lightColor = light->getColor() * light->getIntensity();
                    const float SHADOW_BOUNCE = 0.10f;
                    result = result + (diffuseColor * lightColor * NdotL * SHADOW_BOUNCE);
                    continue;
                }

                Vec3 R = (N * (2.0f * NdotL)) - L;
                R.normalize();
                float RdotV = std::max(R.dot(view_dir), 0.0f);
                float specularFactor = std::pow(RdotV, material.getShininess());
                Color lightColor = light->getColor() * light->getIntensity();
                result = result + (diffuseColor * lightColor * NdotL); // diffuse contribution
                result = result + (material.getSpecular() * lightColor * specularFactor);
            }
            result.clamp(); // make sure color values are within valid range
            return result;
        }

        Color illuminate(const IntersectData& data, const std::vector<std::unique_ptr<Light>>& lights,
            const std::vector<std::unique_ptr<Object>>& objects,
            const Material& material, const Vec3& view_dir, const Texture* texture = nullptr) const override {
            Color localColor = computeLocalIllumination(data, lights, objects, material, view_dir, texture);
            Color reflectedColor(0, 0, 0);
            return combineWithReflection(localColor, material, reflectedColor);
        }
};

// FOR GPU RENDERING
struct LightData {
    Point position;
    Color color;
    float intensity;
    
    CUDA_CALLABLE LightData() : position(Point()), color(Color(255, 255, 255)), intensity(1.0f) {}
    CUDA_CALLABLE LightData(const Point& pos, const Color& col, float inten) 
        : position(pos), color(col), intensity(inten) {}
};

CUDA_CALLABLE inline Color computePhongIllumination(
    const Point& hit_point,
    const Vec3& normal,
    const Vec3& view_dir,
    const Material& material,
    const Color& ambientLight,
    const LightData* lights,
    int numLights
) {
    Color diffuseColor = material.getDiffuse();
    
    Color result = material.getAmbient() * ambientLight;
    for (int i = 0; i < numLights; i++) {
        Vec3 L = lights[i].position - hit_point;
        L.normalize();
        Vec3 N = normal;
        float NdotL = N.dot(L);
        if (NdotL < 0.0f) NdotL = 0.0f;
        Vec3 R = (N * (2.0f * NdotL)) - L;
        R.normalize();
        float RdotV = R.dot(view_dir);
        if (RdotV < 0.0f) RdotV = 0.0f;
        float specularFactor = 1.0f;
        if (RdotV > 0.0f) {
            #ifdef __CUDACC__
                specularFactor = powf(RdotV, material.getShininess());
            #else
                specularFactor = std::pow(RdotV, material.getShininess());
            #endif
        }
        Color lightColor = lights[i].color * lights[i].intensity;
        result = result + (diffuseColor * lightColor * NdotL);
        result = result + (material.getSpecular() * lightColor * specularFactor);
    }
    result.clamp();
    return result;
}

CUDA_CALLABLE inline Color computePhongIlluminationWithTexture(
    const Point& hit_point,
    const Vec3& normal,
    const Vec3& view_dir,
    const Material& material,
    const Color& ambientLight,
    const LightData* lights,
    int numLights,
    const TextureData* texture,
    Color sampledTexture
) {
    // sample texture and modulate diffuse color
    // For procedural textures, use the sampled color directly
    Color diffuseColor = sampledTexture;
    
    Color result = material.getAmbient() * ambientLight;
    for (int i = 0; i < numLights; i++) {
        Vec3 L = lights[i].position - hit_point;
        L.normalize();
        Vec3 N = normal;
        float NdotL = N.dot(L);
        if (NdotL < 0.0f) NdotL = 0.0f;
        Vec3 R = (N * (2.0f * NdotL)) - L;
        R.normalize();
        float RdotV = R.dot(view_dir);
        if (RdotV < 0.0f) RdotV = 0.0f;
        float specularFactor = 1.0f;
        if (RdotV > 0.0f) {
            #ifdef __CUDACC__
                specularFactor = powf(RdotV, material.getShininess());
            #else
                specularFactor = std::pow(RdotV, material.getShininess());
            #endif
        }
        Color lightColor = lights[i].color * lights[i].intensity;
        result = result + (diffuseColor * lightColor * NdotL);
        result = result + (material.getSpecular() * lightColor * specularFactor);
    }
    result.clamp();
    return result;
}

#undef CUDA_CALLABLE

#endif // ILLUMINATION_H
