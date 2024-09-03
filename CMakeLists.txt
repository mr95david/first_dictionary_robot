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
  "srv/GetStatus.srv"
  "srv/SetStatus.srv"
  "srv/ValidateGoal.srv"
  "msg/StateActionC.msg"
 # DEPENDENCIES geometry_msgs
)

# Creacion de nodos de ros2 python
install(PROGRAMS
  ${PROJECT_NAME}/move_pose.py
  ${PROJECT_NAME}/relative_pose.py
  ${PROJECT_NAME}/robot_state.py
  ${PROJECT_NAME}/stop_all.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalacion de carpetas extra, entre estas es la carpetas completmentarias de configuracion
# launch y parametros
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# # Generador de interfaces
# rosidl_generate_interfaces(${PROJECT_NAME}
  
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
