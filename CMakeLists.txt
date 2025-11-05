cmake_minimum_required(VERSION 3.8)
project(cam_package)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)



#for viewer_node
install(PROGRAMS src/viewer_node.py DESTINATION lib/${PROJECT_NAME})


#for parameter_setter
add_executable(parameter_setter src/parameter_setter.cpp)
ament_target_dependencies(parameter_setter
    rclcpp
    rcl_interfaces
    )
install(TARGETS parameter_setter DESTINATION lib/${PROJECT_NAME})


# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()