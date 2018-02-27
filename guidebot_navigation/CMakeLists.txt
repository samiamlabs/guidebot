cmake_minimum_required(VERSION 2.8.3)
project(guidebot_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    tf
    nav_core
    costmap_2d
    base_local_planner
    pluginlib
)

catkin_python_setup()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES linefollow_planner
  CATKIN_DEPENDS
    roscpp
    pluginlib
    costmap_2d
    base_local_planner
    nav_core
)

add_library(linefollow_planner src/linefollow_planner.cpp)
add_dependencies(linefollow_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(linefollow_planner
  ${catkin_LIBRARIES}
)

install(TARGETS linefollow_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES linefollow_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
