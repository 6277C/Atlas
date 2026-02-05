#pragma once

#include "config/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include <cstdint>
#include <math.h>
#include <vector>
namespace atlas {
    float avg(std::vector<float> values);
    template <typename T> constexpr T sgn(T value) { return value < 0 ? -1 : 1; }
    constexpr float radToDeg(float rad) { return rad * 180 / M_PI; }
    constexpr float degToRad(float deg) { return deg * M_PI / 180; }
    float angleError(float target, float position, bool radians = true,
                 AngularDirection direction = AngularDirection::AUTO);
    constexpr float sanitizeAngle(float angle, bool radians = true);
    double linearToRPM(double linearVel,
                   double wheelDiameter,
                   double gearRatio,
                   double maxRPM = 600.0);    
    class Timer{
        public:
        Timer(uint32_t time);
        uint32_t getTimeSet();
        uint32_t getTimeLeft();
        uint32_t getTimePassed();
        bool isDone();
        bool isPaused();
        void set(uint32_t time);
        void reset();
        void pause();
        void resume();
        void waitUntilDone();
        private:
        uint32_t period;
        uint32_t lastTime;
        uint32_t timeWaited = 0;
        bool paused = false;

    };
}