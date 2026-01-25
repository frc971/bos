#pragma once

#include "src/utils/pch.h"

template <typename T>
auto camera_matrix_from_json(nlohmann::json intrinsics) -> T;

template <typename T>
auto distortion_coefficients_from_json(nlohmann::json intrinsics) -> T;
