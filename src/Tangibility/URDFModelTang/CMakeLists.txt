cmake_minimum_required(VERSION 2.8.3)

project(URDFModelTang)

find_package(catkin REQUIRED)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/manipulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
