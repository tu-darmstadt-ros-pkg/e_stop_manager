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
# Resolve validator target robustly
# ------------------------------------------------------------------------------
if(TARGET generate_parameter_lib_validators::generate_parameter_lib_validators)
  set(GPLV_TARGET
      generate_parameter_lib_validators::generate_parameter_lib_validators)
elseif(TARGET generate_parameter_lib_validators)
  set(GPLV_TARGET generate_parameter_lib_validators)
else()
  message(
    FATAL_ERROR
      "generate_parameter_lib_validators found but no usable CMake target exported"
  )
endif()

get_target_property(GPLV_INCLUDE_DIRS ${GPLV_TARGET}
                    INTERFACE_INCLUDE_DIRECTORIES)
list(GET GPLV_INCLUDE_DIRS 0 GPLV_INCLUDE_DIR)

set(GPLV_VALIDATOR_HEADER
    "${GPLV_INCLUDE_DIR}/generate_parameter_lib_validators/parameter_validators.hpp"
)

if(NOT EXISTS "${GPLV_VALIDATOR_HEADER}")
  message(FATAL_ERROR "Validator header not found at ${GPLV_VALIDATOR_HEADER}")
endif()

# ------------------------------------------------------------------------------
# Generate parameter library WITH validator header
# ------------------------------------------------------------------------------
generate_parameter_library(
  e_stop_manager_parameters config/e_stop_parameter_config.yaml
  ${GPLV_VALIDATOR_HEADER})

target_link_libraries(e_stop_manager_parameters INTERFACE ${GPLV_TARGET})

# ------------------------------------------------------------------------------
# Main library
# ------------------------------------------------------------------------------
# Main library
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
# Install
# ------------------------------------------------------------------------------
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_package()
