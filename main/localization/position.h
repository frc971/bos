#pragma once
namespace Localization {
typedef struct Point3d {
  double x;
  double y;
  double z;
} point3d_t;

typedef struct Position {
  point3d_t translation;  // Meters
  point3d_t rotation;     // Radians
  int tag_id;
} position_t;
}  // namespace Localization
