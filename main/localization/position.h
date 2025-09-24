#pragma once
#include <cmath>
#include <ostream>
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
  double distance;
  int tag_id;
  friend std::ostream& operator<<(std::ostream& os, const TagDetection& t) {
    os << "id: " << t.tag_id << "\n";
    os << "Translation: "
                << "\n";
    os << t.translation.x << "\n";
    os << t.translation.y << "\n";
    os << t.translation.z << "\n";
    os << "Rotation: "
       << "\n";
    os << t.rotation.x * 180 / M_PI << "\n";
    os << t.rotation.x * 180 / M_PI << "\n";
    os << t.rotation.x * 180 / M_PI << "\n";
    return os;
  }
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
