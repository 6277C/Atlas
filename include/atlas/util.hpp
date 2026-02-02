#pragma once

#include <math.h>
#include <vector>
namespace atlas {
    float avg(std::vector<float> values);
    template <typename T> constexpr T sgn(T value) { return value < 0 ? -1 : 1; }
    constexpr float radToDeg(float rad) { return rad * 180 / M_PI; }
    constexpr float degToRad(float deg) { return deg * M_PI / 180; }
}