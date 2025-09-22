#pragma once
namespace localization {
typedef struct Point3d {
  double x;
  double y;
  double z;
} point3d_t;

typedef struct TagDetection {
  point3d_t translation;  // Meters
  point3d_t rotation;     // Radians
  double timestamp;
  int tag_id;
} tag_detection_t;

typedef struct Pose2d {
  double x;
  double y;
  double rotation;
} pose2d_t;

typedef struct Pose3d {
  point3d_t translation;
  point3d_t rotation;
} pose3d_t;
}  // namespace localization
