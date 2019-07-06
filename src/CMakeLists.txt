cmake_minimum_required(VERSION 2.8.3)
project(d-orca)

add_compile_options(-std=c++11)

add_library(Simulator Simulator.cpp )
target_link_libraries(Simulator ${catkin_LIBRARIES} Agent rvo2_3d ${GAZEBO_LIBRARIES})
add_dependencies(Simulator ${catkin_EXPORTED_TARGETS})

add_library(Agent Agent.cpp )
target_link_libraries(Agent ${catkin_LIBRARIES} rvo2_3d ${GAZEBO_LIBRARIES})
add_dependencies(Agent ${catkin_EXPORTED_TARGETS})

add_executable(state_publisher publisher.cpp )
target_link_libraries(state_publisher ${catkin_LIBRARIES})
add_dependencies(state_publisher ${catkin_EXPORTED_TARGETS})
