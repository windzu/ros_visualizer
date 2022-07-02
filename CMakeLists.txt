cmake_minimum_required(VERSION 3.5)
project(wind_visualizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# eigen3
find_package(Eigen3 QUIET)

if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# opencv
find_package(OpenCV REQUIRED)

# ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  autoware_msgs
  cv_bridge
  geometry_msgs
  image_transport
  tf
  visualization_msgs
)


catkin_package()


include_directories( ${catkin_INCLUDE_DIRS} )
include_directories(./include)
aux_source_directory(./src SRC)
add_executable(${PROJECT_NAME} ${SRC})

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${OpenCV_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}  
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
