#pragma once

#include <algorithm>
#include <cmath>
namespace atlas {
  //BoxMap testMap(3.556f, 3.556f); // field converted to meters
class FieldMap {
public:
    // inches and radians
  float raycast(float x, float y, float angle) const;
  bool isObstacle(float x, float y) const;
  
};

class BoxMap : public FieldMap {
public:
    float width;
    float height;

    BoxMap(float w, float h) : width(w), height(h) {}

    float raycast(float x, float y, float angle) const {
        float dx = std::sin(angle); // X-right
        float dy = std::cos(angle); // Y-forward
        float tMin = 1e6f;          // large finite number

        float halfW = width / 2.0f;
        float halfH = height / 2.0f;

        // Left wall (x = -halfW)
        if (dx < 0) {
            float t = (-halfW - x) / dx;
            float yHit = y + t * dy;
            if (t > 0 && yHit >= -halfH && yHit <= halfH)
                tMin = std::min(tMin, t);
        }

        // Right wall (x = +halfW)
        if (dx > 0) {
            float t = (halfW - x) / dx;
            float yHit = y + t * dy;
            if (t > 0 && yHit >= -halfH && yHit <= halfH)
                tMin = std::min(tMin, t);
        }

        // Bottom wall (y = -halfH)
        if (dy < 0) {
            float t = (-halfH - y) / dy;
            float xHit = x + t * dx;
            if (t > 0 && xHit >= -halfW && xHit <= halfW)
                tMin = std::min(tMin, t);
        }

        // Top wall (y = +halfH)
        if (dy > 0) {
            float t = (halfH - y) / dy;
            float xHit = x + t * dx;
            if (t > 0 && xHit >= -halfW && xHit <= halfW)
                tMin = std::min(tMin, t);
        }

        return tMin;
    }
};

extern BoxMap testMap;
} // namespace atlas