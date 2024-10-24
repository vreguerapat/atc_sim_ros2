cmake_minimum_required(VERSION 3.8)
project(atc_sim_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

set(msg_files
	"msg/Flight.msg"
	"msg/ListaAviones.msg"
	"msg/Waypoint.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

find_package(rclcpp REQUIRED)

add_executable(aeropuerto src/main_aeropuerto.cpp src/aeropuerto.cpp src/avion.cpp)
ament_target_dependencies(aeropuerto rclcpp geometry_msgs tf2 visualization_msgs)

#Compilar visualizador
add_executable(visualizador src/main_visualizador.cpp src/visualizador.cpp)
ament_target_dependencies(visualizador rclcpp geometry_msgs tf2 visualization_msgs)

include_directories(include)
#Instalar nodos
install(TARGETS
    aeropuerto
    visualizador
  DESTINATION lib/${PROJECT_NAME})
  
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(aeropuerto "${cpp_typesupport_target}")
target_link_libraries(visualizador "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

