#include "atlas/localization/fieldMap.hpp"

namespace atlas {
    constexpr float fieldSize = 140; // inches
    constexpr float halfField = fieldSize / 2.0f;
    BoxMap testMap(fieldSize, fieldSize);

    /**
    * @brief Caculated the length of a simulated distance sensor ray 
    *
    * @param x X value of sensor
    * @param y Y value of sensor
    * @param angle The angle in radians
    */
    float FieldMap::raycast(float x, float y, float angle) const {
    const float step = 0.1f;       // step size in inches (smaller = more precise)
    const float maxDistance = fieldSize; // maximum distance across the field
    float distance = 0.0f;

    while (distance < maxDistance) {
        // Compute current point along the ray
        float currX = x + distance * std::sin(angle); // X-right
        float currY = y + distance * std::cos(angle); // Y-forward (north)

        // Stop if outside field boundaries
        if (currX < -halfField || currX > halfField || currY < -halfField || currY > halfField) {
            return distance;
        }

        // Stop if hits obstacle (add your map later)
        if (isObstacle(currX, currY)) {
            return distance;
        }

        distance += step;
    }

    return maxDistance; // never infinity
}



/**
 * @brief Check if a obstacle is blocking the ray
 * 
 * @param x Value of ray
 * @param y Value of ray
 * @return true If ray is outside of map boundary
 * @return false If no obstacle is hit
 */
bool FieldMap::isObstacle(float x, float y) const {
    float halfField = fieldSize / 2.0f;
    // TODO: replace with actual map check
    // Example: return true if x or y is outside map boundaries
    return x <= -halfField || x >= halfField || y <= -halfField || y >= halfField;
    // Or check your grid/map array
    // return map[int(y)][int(x)] == OBSTACLE;
    return false;
}
}