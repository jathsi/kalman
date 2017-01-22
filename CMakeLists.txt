cmake_minimum_required(VERSION 2.8.3)
project(kalman)

find_package(catkin REQUIRED
    COMPONENTS
        angles
        cmake_modules
        roscpp
        sensor_msgs
        tf
        tf2
        cmake_modules
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(Eigen REQUIRED)

#catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kalman
  DEPENDS Boost Eigen
)

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_library(kalman_def src/kalman.cpp)
target_link_libraries(kalman_def ${catkin_LIBRARIES})
add_library(Roshdl src/Roshdl.cpp)
target_link_libraries(Roshdl kalman_def ${catkin_LIBRARIES})
add_executable(line_new src/therunfile.cpp)
target_link_libraries(line_new kalman_def Roshdl)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(line_new ${catkin_EXPORTED_TARGETS})
endif()



install(TARGETS line_new
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY include/kalman
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")


