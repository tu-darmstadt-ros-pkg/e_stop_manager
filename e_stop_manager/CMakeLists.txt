cmake_minimum_required(VERSION 3.10)
project(e_stop_manager VERSION 1.0.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(e_stop_manager_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

set(CMAKE_CXX_STANDARD 14)

###########
## Build ##
###########

include_directories(include)

# library
add_library(${PROJECT_NAME} SHARED
        src/e_stop_manager.cpp)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        e_stop_manager_msgs
        rclcpp_components
        std_msgs
        )
rclcpp_components_register_nodes(${PROJECT_NAME} "e_stop_manager::EStopManager")


# node        
add_executable(${PROJECT_NAME}_node src/e_stop_manager_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        )
ament_target_dependencies(${PROJECT_NAME}_node
        "rclcpp")
#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#############
## Install ##
#############

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
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
#if(BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    find_package(ament_cmake_pytest REQUIRED)
#
#    ament_add_gtest_executable(${PROJECT_NAME}_test test/test_e_stop_manager.cpp)
#
#    target_include_directories(${PROJECT_NAME}_test PUBLIC
#            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#            $<INSTALL_INTERFACE:include>
#            )
#    ament_target_dependencies(${PROJECT_NAME}_test
#            std_msgs
#            )
#    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
#    # gtest binaries are to be found in the build tree.
#    ament_add_pytest_test(test_gtest_py "test/test_e_stop_manager.launch.py"
#            PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
#            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
#            )
#endif()
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ros_testing REQUIRED)
    ament_add_gtest_executable(${PROJECT_NAME}_test
            test/test_e_stop_manager.cpp
            )
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    ament_target_dependencies(${PROJECT_NAME}_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
    add_ros_test(test/test_e_stop_manager.launch.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif ()

ament_package()
