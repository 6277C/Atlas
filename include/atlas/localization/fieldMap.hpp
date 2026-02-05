#pragma once

namespace atlas {

class FieldMap {
public:
    // inches and radians
  float raycast(float x, float y, float angle) const;
};
} // namespace atlas