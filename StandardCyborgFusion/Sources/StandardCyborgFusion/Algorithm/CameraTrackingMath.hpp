#pragma once
#include <cmath>

inline double cameraFrameInterval(double previous, double current) {
    if (!std::isfinite(previous) || !std::isfinite(current) || current <= previous) return NAN;
    return current - previous;
}

inline float cameraAxisAngle(float cosine) {
    if (!std::isfinite(cosine)) return NAN;
    // Rigid matrix multiplication can round an axis dot product just outside [-1, 1].
    return std::acos(std::fmax(-1.0f, std::fmin(1.0f, cosine)));
}
