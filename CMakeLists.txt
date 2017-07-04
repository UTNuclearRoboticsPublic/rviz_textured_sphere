cmake_minimum_required(VERSION 2.8.3)
project(rviz_textured_sphere)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs sensor_msgs shape_msgs rviz pluginlib class_loader cv_bridge message_generation geometry_msgs sensor_msgs cmake_modules roslint rviz)

#find_package(Eigen REQUIRED)
#find_package(OpenGL REQUIRED)

roslint_cpp()
roslint_python()

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I (original author) prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

# include_directories(SYSTEM "msg_gen/cpp/include")

# add_message_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   sensor_msgs
# )


catkin_package(
)
#   LIBRARIES
#     rviz_textured_quads
#     ${OPENGL_LIBRARIES}
#   INCLUDE_DIRS
#   	src
#     ${OPENGL_INCLUDE_DIR}
#   CATKIN_DEPENDS
#     roscpp
#     rospy
#     roslib
#     std_msgs
#     sensor_msgs
#     shape_msgs
#     rviz
#     pluginlib
#     class_loader
#     cv_bridge
#   	geometry_msgs
#   	sensor_msgs
#   	eigen_conversions
#   	rviz
# )

include_directories(
	src
	${catkin_INCLUDE_DIRS})
	#	${Eigen_INCLUDE_DIRS})

if(rviz_QT_VERSION VERSION_LESS "5")
qt4_wrap_cpp(MOC_FILES
  include/rviz_textured_sphere/sphere_display.h
  )
else()
qt5_wrap_cpp(MOC_FILES
  include/rviz_textured_sphere/sphere_display.h
  )
endif()

include_directories(include ${catkin_INCLUDE_DIRS})

set(TEXTURED_SPHERE_LIB rviz_textured_sphere)

add_library(${TEXTURED_SPHERE_LIB} src/sphere_display.cpp ${MOC_FILES})
target_link_libraries(${TEXTURED_SPHERE_LIB} ${catkin_LIBRARIES} ${QT_LIBRARIES} ) #${Eigen_LIBRARIES}

add_dependencies(${TEXTURED_SPHERE_LIB} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${TEXTURED_SPHER_LIB}  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
