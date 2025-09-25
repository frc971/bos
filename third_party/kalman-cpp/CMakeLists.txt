add_library(kalman-cpp kalman.cc)
target_link_libraries(kalman-cpp PRIVATE Eigen3::Eigen)
