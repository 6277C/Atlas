#include <vector>
#include "atlas/util.hpp"
float atlas::avg(std::vector<float> values) {
    float sum = 0;
    for (float value : values) { sum += value; }
    return sum / values.size();
}
