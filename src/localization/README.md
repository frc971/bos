# Localization Module

We use AprilTags for localization.

---

## Base Interface

### `position_solver.h`
Interface for converting tag detections into robot pose estimates.

---

## Implementations

### `square_solver.{cc,h}`
Uses OpenCV's IPPE_SQUARE solver. Takes advantage of the fact that all object points are planar and a set distance and angle away from each other, so produces the best estimates for individual tag detections.

### `multi_tag_solver.{cc,h}`
Uses in multiple tags at a time, uses a different OpenCV algorithm because the points aren't all planar anymore once we pass in multiple tags. Can only be run on a per-camera basis because OpenCV limits what kinds of matrices you can pass into its functions, so we didn't figure out how to solve using all detections from all cameras at once using OpenCV.

### `joint_solver.{cc,h}`
Custom iterative solver using multiple tag estimates.

The normal PnP problem is:

```

image_point · scalar = image_to_camera · camera_to_world · world_relative_points;

```

In our scenario this is:

```

image_point · scalar = image_to_camera · camera_to_tag · tag_to_field · tag_center_to_tag_corner;

```

This can be rewritten as:

```

image_point · scalar = image_to_camera · camera_to_robot · robot_to_field · field_to_tag · tag_center_to_tag_corner;

```

And simplified to:

```

image_point · scalar = image_to_robot · robot_to_field · field_relative_tag_corner;

```

For an iterative solver, you would normally just compute the gradient here, but if you directly applied the gradient update to the rotation matrix, it wouldn't be guaranteed to be a rotation matrix. We therefore split up the transformation matrix robot_to_field into translation and individual Euler angles and update with respect to those variables instead.

After decomposition the math is effectively:

```

image_to_robot · translation · rotation_z · rotation_y · rotation_x · field_to_tag_center · tag_center_to_tag_corner = projected_corner_in_image

```

We treat like it is a neural network, with the layers being:

```

input/object_point → rotation_x → rotation_y → rotation_z → translation → image_to_robot = projection

```

If image_to_robot being at the end seems confusing, from_image_to_robot really means the robot's position expressed in image coordinates, meaning that multiplying image_to_robot by a matrix in robot coordinates takes it from robot coordinates to image coordinates (the projection).
```
