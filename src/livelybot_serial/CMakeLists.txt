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
    include/hardware/robot.h
    include/hardware/motor.h
    include/robot_node.h
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
add_library(${PROJECT_NAME} ${serial_SRCS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME} rt pthread ${SERIALPORT_LIBRARY})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# test
add_executable(ik_run test/test_ik_run.cpp ${serial_SRCS})
add_dependencies(ik_run ${PROJECT_NAME})
target_link_libraries(ik_run ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(robot_node_ src/robot_node.cpp ${serial_SRCS})
add_dependencies(robot_node_ ${PROJECT_NAME})
target_link_libraries(robot_node_ ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_motor test/test_motor.cpp ${serial_SRCS})
add_dependencies(test_motor ${PROJECT_NAME})
target_link_libraries(test_motor ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_reset_zero test/test_reset_zero.cpp ${serial_SRCS})
add_dependencies(test_reset_zero ${PROJECT_NAME})
target_link_libraries(test_reset_zero ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_feedback test/test_feedback.cpp ${serial_SRCS})
add_dependencies(test_feedback ${PROJECT_NAME})
target_link_libraries(test_feedback ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_motor_feedback test/test_motor_feedback.cpp ${serial_SRCS})
add_dependencies(test_motor_feedback ${PROJECT_NAME})
target_link_libraries(test_motor_feedback ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_motor_run test/test_motor_run.cpp ${serial_SRCS})
add_dependencies(test_motor_run ${PROJECT_NAME})
target_link_libraries(test_motor_run ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

add_executable(test_reconfigure test/test_reconfigure.cpp ${serial_SRCS})
add_dependencies(test_reconfigure ${PROJECT_NAME})
target_link_libraries(test_reconfigure ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install headers
install(FILES include/hardware/robot.h include/hardware/motor.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/livelybot_serial)
