cmake_minimum_required(VERSION 2.8.3)
project(my_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  control_toolbox
  #forward_command_controller
  realtime_tools
  urdf
  
  message_generation
  message_runtime
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
#include_directories(include ${catkin_INCLUDE_DIRS})

 add_message_files(
   FILES
   JointControllerState.msg
   JointError.msg
#   Message2.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    control_toolbox
    #forward_command_controller
    realtime_tools
    urdf
    
    message_generation
    message_runtime

  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )


add_library(${PROJECT_NAME}

  src/joint_position_controller.cpp
  src/joint_controller.cpp
  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})





# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES my_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


