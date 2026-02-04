#pragma once

struct PIDConstants {
    float kP;
    float kI;
    float kD;
    float antiWindup;
};

extern PIDConstants relativeAngularParams;
extern PIDConstants relativeLinearParams;
extern PIDConstants globalAngularParams;
extern PIDConstants globalLinearParams;