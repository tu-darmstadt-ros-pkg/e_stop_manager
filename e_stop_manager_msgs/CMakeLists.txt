cmake_minimum_required(VERSION 3.10)
project(e_stop_manager_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

###########
## Build ##
###########

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/EStopList.msg"
        "srv/SetEStop.srv"
        )

#############
## Install ##
#############

install(DIRECTORY "msg" "srv"
        DESTINATION "share/${PROJECT_NAME}"
        )

ament_package()