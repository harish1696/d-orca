cmake_minimum_required(VERSION 2.8.3)
project(d-orca)

add_compile_options(-std=c++11)

add_executable(dorcacircle circle.cpp )
target_link_libraries(dorcacircle ${catkin_LIBRARIES} Simulator Agent rvo2_3d ${GAZEBO_LIBRARIES})
add_dependencies(dorcacircle ${catkin_EXPORTED_TARGETS})