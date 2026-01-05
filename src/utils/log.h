#pragma once
#include <frc/geometry/Pose3d.h>
#include "absl/log/check.h"
#include "absl/log/log.h"

void PrintPose3d(const frc::Pose3d& pose);
void PrintTransform3d(const frc::Transform3d& transform);
