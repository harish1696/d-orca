cmake_minimum_required(VERSION 2.8.3)
project(d-orca)

add_compile_options(-std=c++11)

add_library(rvo2_3d rvo2_3d/Agent.cpp rvo2_3d/KdTree.cpp rvo2_3d/RVOSimulator.cpp rvo2_3d/tiny_obj_loader.cc)
target_link_libraries(rvo2_3d ${catkin_LIBRARIES})
add_dependencies(rvo2_3d ${catkin_EXPORTED_TARGETS})