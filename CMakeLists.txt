project(dorca)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  mavros
  message_generation
)

add_message_files(DIRECTORY msg)

generate_messages( DEPENDENCIES
  std_msgs
	geometry_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
 INCLUDE_DIRS include libs src
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs mavros message_generation
)

include_directories(include
   libs
   src
   ${catkin_INCLUDE_DIRS}
)
include_directories( include ${Boost_INCLUDE_DIR} )
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_subdirectory(libs)
add_subdirectory(src)
add_subdirectory(examples)
