cmake_minimum_required(VERSION 3.0.2)
project(livelybot_serial)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    livelybot_msg  
    serial
    dynamic_reconfigure
)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(serial_SRCS
    src/lively_serial.cc
    src/hardware/motor.cc
    src/hardware/canport.cc
    src/hardware/canboard.cc
    src/hardware/robot.cc
)


generate_dynamic_reconfigure_options(
  cfg/robot_dynamic_config_20.cfg
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS rt pthread
    CATKIN_DEPENDS roscpp std_msgs livelybot_msg serial dynamic_reconfigure
)
find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(livelybot_serial SHARED ${serial_SRCS})
add_dependencies(livelybot_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(livelybot_serial ${catkin_LIBRARIES} rt pthread ${SERIALPORT_LIBRARY})

link_directories(lib)

# # test
add_executable(ik_run test/test_ik_run.cpp)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(ik_run liblivelybot_serial_arm64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(ik_run liblivelybot_serial_x86_64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
endif()

add_executable(test_motor test/test_motor.cpp)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(test_motor liblivelybot_serial_arm64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(test_motor liblivelybot_serial_x86_64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
endif()

add_executable(test_reset_zero test/test_reset_zero.cpp)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(test_reset_zero liblivelybot_serial_arm64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(test_reset_zero liblivelybot_serial_x86_64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
endif()

add_executable(test_feedback test/test_feedback.cpp)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(test_feedback liblivelybot_serial_arm64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(test_feedback liblivelybot_serial_x86_64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
endif()

add_executable(test_motor_feedback test/test_motor_feedback.cpp)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(test_motor_feedback liblivelybot_serial_arm64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(test_motor_feedback liblivelybot_serial_x86_64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
endif()

add_executable(test_motor_run test/test_motor_run.cpp)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(test_motor_run liblivelybot_serial_arm64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(test_motor_run liblivelybot_serial_x86_64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
endif()

add_executable(test_jump test/test_jump.cpp)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(test_jump liblivelybot_serial_arm64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(test_jump liblivelybot_serial_x86_64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
endif()

add_executable(test_reconfigure test/test_reconfigure.cpp)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(test_reconfigure liblivelybot_serial_arm64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(test_reconfigure liblivelybot_serial_x86_64.so ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})
endif()
