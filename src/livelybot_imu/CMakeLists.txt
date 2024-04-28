cmake_minimum_required(VERSION 3.0.2)

project(livelybot_imu)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")  
SET(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  serial
)
include_directories(
  include 
  src
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)
catkin_package(
  CATKIN_DEPENDS
 INCLUDE_DIRS include
 LIBRARIES  ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf serial
#  DEPENDS system_lib
)

set(imu_SRCS
    include/ahrs_driver.h
    include/crc_table.h
    include/fdilink_data_struct.h
    src/ahrs_driver.cpp
    src/crc_table.cpp
)


## ahrs_driver
add_executable(ahrs_driver src/ahrs_driver.cpp)
target_link_libraries(ahrs_driver crc_table ${catkin_LIBRARIES})# ${SERIAL_LIB}
# target_compile_options(ahrs_driver PRIVATE -g)

add_executable(test_eigen src/test_eigen.cpp)
target_link_libraries(test_eigen crc_table ${catkin_LIBRARIES})# ${SERIAL_LIB}
# target_compile_options(test_eigen PRIVATE -g)


add_library(livelybot_imu SHARED ${imu_SRCS})
target_link_libraries(livelybot_imu ${catkin_LIBRARIES})
target_link_directories(livelybot_imu PRIVATE ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} )

# target_compile_options(livelybot_imu PRIVATE -g)


install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install headers
install(FILES include/ahrs_driver.h include/crc_table.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/livelybot_imu)


add_executable(imu_tf src/imu_tf.cpp)
target_link_libraries( imu_tf ${catkin_LIBRARIES})# ${SERIAL_LIB}
## crc_table
add_library(crc_table src/crc_table.cpp)
