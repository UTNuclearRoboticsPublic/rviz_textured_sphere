cmake_minimum_required(VERSION 2.8.3)
project(rviz_textured_sphere)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rviz pluginlib cv_bridge)


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

## Avoid defining "emit", "slots", etc because they can conflict with boost signals.
add_definitions(-DQT_NO_KEYWORDS)
add_compile_options(-std=c++11 -Wall)


catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS})

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
target_link_libraries(${TEXTURED_SPHERE_LIB} ${catkin_LIBRARIES} ${QT_LIBRARIES} )

add_dependencies(${TEXTURED_SPHERE_LIB} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${TEXTURED_SPHER_LIB}  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY ogre_media rviz_cfg launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
