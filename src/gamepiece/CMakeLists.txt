add_library(gamepiece gamepiece.cc ellipse.cc)
target_link_libraries(gamepiece camera yolo utils Eigen3::Eigen)
# Eigen's unsupported polynomial solver currently fails to compile with its
# ARM NEON packet path; this target only operates on tiny fixed-size matrices.
target_compile_definitions(gamepiece PRIVATE EIGEN_DONT_VECTORIZE)
