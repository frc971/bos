
add_library(utils timer.cc nt_utils.cc) 
target_link_libraries(utils PUBLIC wpilibc)
