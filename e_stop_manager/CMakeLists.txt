cmake_minimum_required(VERSION 3.10)
project(e_stop_manager VERSION 1.0.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(e_stop_manager_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 14)

###########
## Build ##
###########

include_directories(include)


add_executable(${PROJECT_NAME}_node src/e_stop_manager_node.cpp src/e_stop_manager.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        e_stop_manager_msgs
        std_msgs
        )
#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#############
## Install ##
#############

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
        )

#[[
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )
]]

ament_package()
