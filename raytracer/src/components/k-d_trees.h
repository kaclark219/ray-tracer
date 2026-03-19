/*
need to read in complex scene from some file format, build the k-d tree, & traverse it when spawning rays
notes for building k-d trees:
    - cuts are made along primary axes (x,y,z)
    - each node contains a bounding box that encompasses all the geometry in its subtree
    - leaf nodes contain a small number of primitives (e.g., triangles)
    - need interior node (subdivision plane axis & value, front & rear children) & leaf node structures (pointer to object(s) found in voxel)
    - decide termination by # of objects in voxel or by depth of tree
    - find partition plane by using spatial median in a round robin fashion
    - partition primitives by testing their bounding boxes against the partition plane
        - primitives can be triangle, AABB, sphere, etc.
        - before you put an object in a k-d tree, but it in a box .. test boxes within boxes
    - needs to be part of world building process, not ray tracing process
    - if interior node, recurse until you get to leaf node, then test ray against all primitives in leaf node
        - will be one of four cases for interior nodes
    - TA-B algorithm makes use of coordinates for entry, exit, & the plane intersection point of axis to traverse the tree efficiently
    - add code to measure the runtime of building the k-d tree & the ray tracing process with & without the k-d tree to show the improvement
    - tinyobjloader for reading in .obj files to get triangle meshes
*/

#ifndef RAYTRACER_K_D_TREES_H
#define RAYTRACER_K_D_TREES_H

#include "../object.h"
#include "../objects/sphere.h"
#include "../objects/triangle.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

class KDTree {
    public:
        struct AABB {
            Point min;
            Point max;

            AABB()
            : min(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()),
            max(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()) {}

            bool isValid() const {
                return min.getX() <= max.getX() && min.getY() <= max.getY() && min.getZ() <= max.getZ();
            }

            void expand(const Point& p) {
                min.setX(std::min(min.getX(), p.getX()));
                min.setY(std::min(min.getY(), p.getY()));
                min.setZ(std::min(min.getZ(), p.getZ()));

                max.setX(std::max(max.getX(), p.getX()));
                max.setY(std::max(max.getY(), p.getY()));
                max.setZ(std::max(max.getZ(), p.getZ()));
            }

            void expand(const AABB& other) {
                if (!other.isValid()) {
                    return;
                }
                expand(other.min);
                expand(other.max);
            }

            float extent(int axis) const {
                return getAxis(max, axis) - getAxis(min, axis);
            }

            // bool AABB_intersect (A,B) {
            //     for each axis x, y, z:
            //     if (Aaxis.min > Baxis.max or Aaxis.max < Baxis.min) return false
            //     return true
            // }
            bool intersects(const AABB& other) const {
                for (int axis = 0; axis < 3; ++axis) {
                    if (getAxis(min, axis) > getAxis(other.max, axis) ||
                        getAxis(max, axis) < getAxis(other.min, axis)) {
                        return false;
                    }
                }
                return true;
            }

            bool intersectRay(const Ray& ray, float& tEnter, float& tExit) const {
                tEnter = 0.0f;
                tExit = std::numeric_limits<float>::infinity();

                for (int axis = 0; axis < 3; ++axis) {
                    const float origin = getAxis(ray.getOrigin(), axis);
                    const float direction = getAxis(ray.getDirection(), axis);
                    const float minValue = getAxis(min, axis);
                    const float maxValue = getAxis(max, axis);

                    if (std::abs(direction) < 1e-8f) {
                        if (origin < minValue || origin > maxValue) {
                            return false;
                        }
                        continue;
                    }

                    const float invDirection = 1.0f / direction;
                    float t0 = (minValue - origin) * invDirection;
                    float t1 = (maxValue - origin) * invDirection;

                    if (t0 > t1) {
                        std::swap(t0, t1);
                    }

                    tEnter = std::max(tEnter, t0);
                    tExit = std::min(tExit, t1);

                    if (tEnter > tExit) {
                        return false;
                    }
                }

                return tExit >= 0.0f;
            }
        };

        struct HitResult {
            const Object* object = nullptr;
            float t = std::numeric_limits<float>::infinity();
        };

    private:
        struct PrimitiveRef {
            const Object* object = nullptr;
            AABB bounds;
        };

        struct Node {
            AABB bounds;
            int axis = -1;
            float split = 0.0f;
            std::unique_ptr<Node> front;
            std::unique_ptr<Node> rear;
            std::vector<const Object*> primitives;

            bool isLeaf() const {
                return !front && !rear;
            }
        };

        std::unique_ptr<Node> root;
        std::size_t maxLeafPrimitives;
        std::size_t maxDepth;
        double buildMilliseconds;
        std::size_t nodeCount;
        std::size_t leafCount;
        std::size_t unsupportedPrimitiveCount;

    public:
        explicit KDTree(std::size_t leafPrimitiveLimit = 4, std::size_t depthLimit = 24) 
        : root(nullptr), maxLeafPrimitives(leafPrimitiveLimit), maxDepth(depthLimit), buildMilliseconds(0.0), nodeCount(0), leafCount(0), unsupportedPrimitiveCount(0) {}

        // to kick things off:
        // all = list of all primitives
        // bb = AABB of all primitives
        // root = getNode(all, bb)
        void build(const std::vector<std::unique_ptr<Object>>& objects) {
            const auto start = std::chrono::high_resolution_clock::now();

            root.reset();
            nodeCount = 0;
            leafCount = 0;
            unsupportedPrimitiveCount = 0;

            std::vector<PrimitiveRef> allPrimitives;
            allPrimitives.reserve(objects.size());

            AABB sceneBounds;
            for (const auto& object : objects) {
                AABB primitiveBounds;
                if (!computeObjectBounds(*object, primitiveBounds)) {
                    ++unsupportedPrimitiveCount;
                    continue;
                }

                sceneBounds.expand(primitiveBounds);
                allPrimitives.push_back({object.get(), primitiveBounds});
            }

            if (!allPrimitives.empty() && sceneBounds.isValid()) {
                root = buildNode(allPrimitives, sceneBounds, 0, 0);
            }

            const auto end = std::chrono::high_resolution_clock::now();
            buildMilliseconds = std::chrono::duration<double, std::milli>(end - start).count();
        }

        bool intersect(const Ray& ray, HitResult& result) const {
            if (!root) {
                return false;
            }

            float tEnter = 0.0f;
            float tExit = 0.0f;
            if (!root->bounds.intersectRay(ray, tEnter, tExit)) {
                return false;
            }

            result = HitResult();
            return traverseNode(root.get(), ray, tEnter, tExit, result);
        }

        bool occluded(const Ray& ray, float maxDistance) const {
            HitResult result;
            if (!intersect(ray, result)) {
                return false;
            }
            return result.t > 1e-3f && result.t < maxDistance;
        }

        bool isBuilt() const {
            return root != nullptr;
        }

        double getBuildMilliseconds() const {
            return buildMilliseconds;
        }

        std::size_t getNodeCount() const {
            return nodeCount;
        }

        std::size_t getLeafCount() const {
            return leafCount;
        }

        std::size_t getUnsupportedPrimitiveCount() const {
            return unsupportedPrimitiveCount;
        }

        const AABB* getBounds() const {
            return root ? &root->bounds : nullptr;
        }

    private:
        // Node getNode (List of primitives L, Voxel V) {
        //     if (Terminal(L,V)) return new leaf node (L)
        //     find partition plane P
        //     split V with P producing Vfront & Vrear
        //     partition elements of L producing Lfront & Lrear
        //     return new interior node (P, getNode(Lfront, Vfront), getNode(Lrear, Vrear))
        // }
        std::unique_ptr<Node> buildNode(const std::vector<PrimitiveRef>& primitives, const AABB& voxel, std::size_t depth, int axis) {
            auto node = std::make_unique<Node>();
            node->bounds = voxel;
            ++nodeCount;

            // decide termination by # of objects in voxel or by depth of tree
            if (shouldTerminate(primitives, voxel, depth)) {
                node->primitives.reserve(primitives.size());
                for (const PrimitiveRef& primitive : primitives) {
                    node->primitives.push_back(primitive.object);
                }
                ++leafCount;
                return node;
            }

            // cuts are made along primary axes (x,y,z)
            // find partition plane by using spatial median in a round robin fashion
            node->axis = axis;
            node->split = 0.5f * (getAxis(voxel.min, axis) + getAxis(voxel.max, axis));

            // split V with P producing Vfront & Vrear
            AABB frontVoxel = voxel;
            AABB rearVoxel = voxel;
            setAxis(frontVoxel.min, axis, node->split);
            setAxis(rearVoxel.max, axis, node->split);

            // partition primitives by testing their bounding boxes against the partition plane
            //     - primitives can be triangle, AABB, sphere, etc.
            //     - before you put an object in a k-d tree, but it in a box .. test boxes within boxes
            std::vector<PrimitiveRef> frontPrimitives;
            std::vector<PrimitiveRef> rearPrimitives;
            frontPrimitives.reserve(primitives.size());
            rearPrimitives.reserve(primitives.size());

            for (const PrimitiveRef& primitive : primitives) {
                if (primitive.bounds.intersects(frontVoxel)) {
                    frontPrimitives.push_back(primitive);
                }
                if (primitive.bounds.intersects(rearVoxel)) {
                    rearPrimitives.push_back(primitive);
                }
            }

            const bool splitFailed =
                frontPrimitives.empty() ||
                rearPrimitives.empty() ||
                (frontPrimitives.size() == primitives.size() && rearPrimitives.size() == primitives.size());

            if (splitFailed) {
                node->axis = -1;
                node->split = 0.0f;
                node->primitives.reserve(primitives.size());
                for (const PrimitiveRef& primitive : primitives) {
                    node->primitives.push_back(primitive.object);
                }
                ++leafCount;
                return node;
            }

            // need interior node (subdivision plane axis & value, front & rear children) & leaf node structures (pointer to object(s) found in voxel)
            node->front = buildNode(frontPrimitives, frontVoxel, depth + 1, (axis + 1) % 3);
            node->rear = buildNode(rearPrimitives, rearVoxel, depth + 1, (axis + 1) % 3);
            return node;
        }

        bool shouldTerminate(const std::vector<PrimitiveRef>& primitives, const AABB& voxel, std::size_t depth) const {
            if (primitives.size() <= maxLeafPrimitives || depth >= maxDepth) {
                return true;
            }

            const float minExtent = std::min({voxel.extent(0), voxel.extent(1), voxel.extent(2)});
            return minExtent <= 1e-5f;
        }

        // if interior node, recurse until you get to leaf node, then test ray against all primitives in leaf node
        //     - will be one of four cases for interior nodes
        // TA-B algorithm makes use of coordinates for entry, exit, & the plane intersection point of axis to traverse the tree efficiently
        bool traverseNode(const Node* node, const Ray& ray, float tEnter, float tExit, HitResult& result) const {
            if (!node) {
                return false;
            }

            if (node->isLeaf()) {
                bool hitAnything = false;
                for (const Object* object : node->primitives) {
                    float t = 0.0f;
                    if (object->intersect(ray, t) && t > 1e-6f && t < result.t) {
                        result.object = object;
                        result.t = t;
                        hitAnything = true;
                    }
                }
                return hitAnything;
            }

            float frontEnter = 0.0f;
            float frontExit = 0.0f;
            float rearEnter = 0.0f;
            float rearExit = 0.0f;

            const bool hitsFront = node->front && node->front->bounds.intersectRay(ray, frontEnter, frontExit);
            const bool hitsRear = node->rear && node->rear->bounds.intersectRay(ray, rearEnter, rearExit);

            if (!hitsFront && !hitsRear) {
                return false;
            }

            const Node* firstChild = nullptr;
            const Node* secondChild = nullptr;
            float firstEnter = 0.0f;
            float firstExit = 0.0f;
            float secondEnter = 0.0f;
            float secondExit = 0.0f;

            if (hitsFront && (!hitsRear || frontEnter <= rearEnter)) {
                firstChild = node->front.get();
                firstEnter = frontEnter;
                firstExit = frontExit;
                secondChild = node->rear.get();
                secondEnter = rearEnter;
                secondExit = rearExit;
            } else {
                firstChild = node->rear.get();
                firstEnter = rearEnter;
                firstExit = rearExit;
                secondChild = node->front.get();
                secondEnter = frontEnter;
                secondExit = frontExit;
            }

            bool hitFirst = false;
            if (firstChild) {
                hitFirst = traverseNode(firstChild, ray, firstEnter, firstExit, result);
                if (hitFirst && result.t <= firstExit) {
                    return true;
                }
            }

            bool hitSecond = false;
            if (secondChild) {
                hitSecond = traverseNode(secondChild, ray, secondEnter, secondExit, result);
            }

            return hitFirst || hitSecond;
        }

        static bool computeObjectBounds(const Object& object, AABB& bounds) {
            if (const auto* sphere = dynamic_cast<const Sphere*>(&object)) {
                const Point center = sphere->getCenter();
                const float radius = sphere->getRadius();
                bounds.expand(Point(center.getX() - radius, center.getY() - radius, center.getZ() - radius));
                bounds.expand(Point(center.getX() + radius, center.getY() + radius, center.getZ() + radius));
                return true;
            }

            if (const auto* triangle = dynamic_cast<const Triangle*>(&object)) {
                bounds.expand(triangle->getVertex(0));
                bounds.expand(triangle->getVertex(1));
                bounds.expand(triangle->getVertex(2));
                return true;
            }

            return false;
        }

        static float getAxis(const Point& point, int axis) {
            if (axis == 0) {
                return point.getX();
            }
            if (axis == 1) {
                return point.getY();
            }
            return point.getZ();
        }

        static float getAxis(const Vec3& vector, int axis) {
            if (axis == 0) {
                return vector.getX();
            }
            if (axis == 1) {
                return vector.getY();
            }
            return vector.getZ();
        }

        static void setAxis(Point& point, int axis, float value) {
            if (axis == 0) {
                point.setX(value);
                return;
            }
            if (axis == 1) {
                point.setY(value);
                return;
            }
            point.setZ(value);
        }
};

#endif // RAYTRACER_K_D_TREES_H
