#ifndef RAYTRACER_PHOTON_MAP_H
#define RAYTRACER_PHOTON_MAP_H

#include "photon.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

class PhotonMap {
    private:
        struct Node {
            const Photon* photon = nullptr;
            int axis = 0;
            std::unique_ptr<Node> left;
            std::unique_ptr<Node> right;
        };

        std::vector<Photon> photons;
        std::unique_ptr<Node> root;

    public:
        PhotonMap() = default;

        void clear() {
            photons.clear();
            root.reset();
        }

        void reserve(std::size_t count) {
            photons.reserve(count);
        }

        void store(const Photon& photon) {
            photons.push_back(photon);
        }

        std::size_t size() const {
            return photons.size();
        }

        bool empty() const {
            return photons.empty();
        }

        const std::vector<Photon>& getPhotons() const {
            return photons;
        }

        std::vector<Photon>& getPhotons() {
            return photons;
        }

        std::vector<const Photon*> gather(const Point& position, float radius) const {
            std::vector<const Photon*> nearby;
            const float radiusSquared = radius * radius;
            gatherNode(root.get(), position, radiusSquared, nearby);
            return nearby;
        }

        void build() {
            root.reset();
            if (photons.empty()) {
                return;
            }
            root = buildNode(0, photons.size(), 0);
        }

    private:
        std::unique_ptr<Node> buildNode(std::size_t begin, std::size_t end, int axis) {
            if (begin >= end) {
                return nullptr;
            }

            const std::size_t mid = begin + (end - begin) / 2;
            std::nth_element(
                photons.begin() + static_cast<std::ptrdiff_t>(begin),
                photons.begin() + static_cast<std::ptrdiff_t>(mid),
                photons.begin() + static_cast<std::ptrdiff_t>(end),
                [axis](const Photon& a, const Photon& b) {
                    return getAxis(a.position, axis) < getAxis(b.position, axis);
                }
            );

            auto node = std::make_unique<Node>();
            node->photon = &photons[mid];
            node->axis = axis;
            node->left = buildNode(begin, mid, (axis + 1) % 3);
            node->right = buildNode(mid + 1, end, (axis + 1) % 3);
            return node;
        }

        void gatherNode(const Node* node, const Point& position, float radiusSquared, std::vector<const Photon*>& nearby) const {
            if (!node || node->photon == nullptr) {
                return;
            }

            const Photon* photon = node->photon;
            const float dx = photon->position.getX() - position.getX();
            const float dy = photon->position.getY() - position.getY();
            const float dz = photon->position.getZ() - position.getZ();
            const float distanceSquared = dx * dx + dy * dy + dz * dz;
            if (distanceSquared <= radiusSquared) {
                nearby.push_back(photon);
            }

            const int axis = node->axis;
            const float delta = getAxis(position, axis) - getAxis(photon->position, axis);
            const float deltaSquared = delta * delta;

            const Node* first = delta <= 0.0f ? node->left.get() : node->right.get();
            const Node* second = delta <= 0.0f ? node->right.get() : node->left.get();

            gatherNode(first, position, radiusSquared, nearby);
            if (deltaSquared <= radiusSquared) {
                gatherNode(second, position, radiusSquared, nearby);
            }
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
};

#endif // RAYTRACER_PHOTON_MAP_H
