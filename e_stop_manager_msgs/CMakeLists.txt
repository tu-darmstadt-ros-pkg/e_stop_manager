cmake_minimum_required(VERSION 3.0.2)
project(e_stop_manager_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  )


add_message_files(FILES
  EStopList.msg
  )

add_service_files(FILES
    SetEStop.srv
  )

generate_messages()


catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

