cmake_minimum_required(VERSION 2.8.3)

project(openni_tracker2)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS geometry_msgs
					roscpp
					roslib
					tf)
# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

catkin_package()

include_directories(${catkin_INCLUDEDIR}
		    ${OpenNI_INCLUDEDIR}
		    ${orocos_kdl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(openni_tracker2 src/openni_tracker2.cpp)

add_dependencies(openni_tracker2 geometry_msgs_gencpp)

target_link_libraries(openni_tracker2 ${catkin_LIBRARIES}
				     ${OpenNI_LIBRARIES}
				     ${orocos_kdl_LIBRARIES})

install(TARGETS openni_tracker2 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES openni_tracker2.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
