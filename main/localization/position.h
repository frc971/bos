#pragma once
namespace Localization {
typedef struct Point3d {
  double x;
  double y;
  double z;
} point3d_t;

typedef struct TagDetection {
  point3d_t translation;  // Meters
  point3d_t rotation;     // Radians
  int tag_id;
} tag_detection_t;

typedef struct Pose2d {
  double x;
  double y;
  double rotation;
} tag_dection_t;

}  // namespace Localization
