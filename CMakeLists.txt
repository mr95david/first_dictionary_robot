cmake_minimum_required(VERSION 3.8)
project(ntarobot_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# Paquete necesario para la instalacion de interfaces de servicio
find_package(rosidl_default_generators REQUIRED)

# Orden de configuracion de cada uno de los servicios
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
  "srv/RelativeMove.srv"
 # DEPENDENCIES geometry_msgs
)

# Creacion de nodos de ros2 python
install(PROGRAMS
  ${PROJECT_NAME}/move_pose.py
  ${PROJECT_NAME}/relative_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
