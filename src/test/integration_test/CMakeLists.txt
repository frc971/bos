add_executable(apriltag_detect_test apriltag_detect_test.cc)
target_link_libraries(apriltag_detect_test PRIVATE cscore camera localization utils)

add_executable(intrinsics_test intrinsics_test.cc)
target_link_libraries(intrinsics_test PRIVATE camera utils)

add_executable(yolo_test yolo_test.cc)
target_link_libraries(yolo_test PRIVATE camera yolo utils)

add_executable(path_plan_test path_plan_test.cc)
target_link_libraries(path_plan_test PRIVATE utils)

add_executable(gamepiece_test gamepiece_test.cc)
target_link_libraries(gamepiece_test PRIVATE gamepiece yolo camera localization utils)

add_executable(solver_test solver_test.cc)
target_link_libraries(solver_test PRIVATE utils localization)

add_executable(localization_test localization_test.cc)
target_link_libraries(localization_test PRIVATE camera localization utils)

add_executable(localization_test2 localization_test2.cc)
target_link_libraries(localization_test2 PRIVATE camera localization utils)

add_executable(networktable_performance_test networktable_performance_test.cc)
target_link_libraries(networktable_performance_test PRIVATE utils localization)

add_executable(pva_test pva_test.cc)
target_link_libraries(pva_test PRIVATE utils localization vpi)

add_executable(auto_check_localization auto_check_localization.cc)
target_link_libraries(auto_check_localization PRIVATE camera localization utils)
