#pragma once

#include <cmath>

inline bool is_zero(float value) { return std::fabsf(value) < 1e-9f; }
