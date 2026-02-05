#pragma once

namespace atlas {
class Pose {
    public:
        float x;
        float y;
        float angle;

        Pose() : x(0), y(0), angle(0) {}
        // create unitless pose
        Pose(float x, float y, float angle = 0);
        // addition
        Pose operator+(const Pose& other) const;
        // subtraction
        Pose operator-(const Pose& other) const;
        // division
        Pose operator/(const float& other) const;
        // multiplication
        Pose operator*(const float& other) const;
        float operator*(const Pose& other) const;
        // linearly interpolate between two points
        Pose lerp(Pose other, float t) const;
        // find distance between two points
        float distance(Pose other) const;
};
} // namespace lemlib