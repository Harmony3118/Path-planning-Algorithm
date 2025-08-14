#pragma once
#include <vector>

inline bool isObstacleTooClose(const std::vector<float>& scan, float threshold = 0.3f) {
    for (float d : scan) {
        if (d < threshold)
            return true;
    }
    return false;
}