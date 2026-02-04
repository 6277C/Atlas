#include <vector>
#include "atlas/util.hpp"
#include "pros/rtos.hpp"

namespace atlas {

float avg(std::vector<float> values) {
    float sum = 0;
    for (float value : values) { sum += value; }
    return sum / values.size();
}

constexpr float sanitizeAngle(float angle, bool radians) {
    if (radians) return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else return std::fmod(std::fmod(angle, 360) + 360, 360);
}

float angleError(float target, float position, bool radians, AngularDirection direction) {
    // bound angles from 0 to 2pi or 0 to 360
    target = sanitizeAngle(target, radians);
    position = sanitizeAngle(position, radians);
    const float max = radians ? 2 * M_PI : 360;
    const float rawError = target - position;
    switch (direction) {
        case AngularDirection::CW: // turn clockwise
            return rawError < 0 ? rawError + max : rawError; // add max if sign does not match
        case AngularDirection::CCW: // turn counter-clockwise
            return rawError > 0 ? rawError - max : rawError; // subtract max if sign does not match
        default: // choose the shortest path
            return std::remainder(rawError, max);
    }
}

Timer::Timer(uint32_t time)
    : period(time) {
    lastTime = pros::millis();
}

uint32_t Timer::getTimeSet() {
    const uint32_t time = pros::millis(); // get time from RTOS
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time
    return period;
}

uint32_t Timer::getTimeLeft() {
    const uint32_t time = pros::millis(); // get time from RTOS
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time
    const int delta = period - timeWaited; // calculate how much time is left
    return (delta > 0) ? delta : 0; // return 0 if timer is done
}

uint32_t Timer::getTimePassed() {
    const uint32_t time = pros::millis(); // get time from RTOS
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time;
    return timeWaited;
}

bool Timer::isDone() {
    const uint32_t time = pros::millis(); // get time from RTOS
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time
    const int delta = period - timeWaited; // calculate how much time is left
    return delta <= 0;
}

bool Timer::isPaused() {
    const uint32_t time = pros::millis(); // get time from RTOS
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    return paused;
}

void Timer::set(uint32_t time) {
    period = time; // set how long to wait
    reset();
}

void Timer::reset() {
    timeWaited = 0;
    lastTime = pros::millis();
}

void Timer::pause() {
    if (!paused) lastTime = pros::millis();
    paused = true;
}

void Timer::resume() {
    if (paused) lastTime = pros::millis();
    paused = false;
}

void Timer::waitUntilDone() {
    do pros::delay(5);
    while (!this->isDone());
}
}