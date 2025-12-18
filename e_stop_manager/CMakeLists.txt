cmake_minimum_required(VERSION 3.10)
project(e_stop_manager VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(e_stop_manager_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(generate_parameter_lib_validators REQUIRED)

# ------------------------------------------------------------------------------
# Generate parameter library WITH validator header
# ------------------------------------------------------------------------------
generate_parameter_library_with_validators(e_stop_manager_parameters
                                           config/e_stop_parameter_config.yaml)

# ------------------------------------------------------------------------------
# Main library
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED src/e_stop_manager.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} e_stop_manager_parameters)

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components std_msgs
                          e_stop_manager_msgs)

# ------------------------------------------------------------------------------
# Node executable
# ------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/e_stop_manager_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  ament_add_gtest_executable(${PROJECT_NAME}_test test/test_e_stop_manager.cpp)

  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs
                            e_stop_manager_msgs)

  add_ros_test(test/test_e_stop_manager.launch.py ARGS
               "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
          e_stop_manager_parameters # <--- ADD THIS LINE
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_export_dependencies(rclcpp rclcpp_components std_msgs e_stop_manager_msgs
                          generate_parameter_lib_validators)

ament_package()
