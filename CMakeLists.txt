cmake_minimum_required(VERSION 3.8)
project(esp_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# LibSerial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# Ejecutable
add_executable(esp_bridge_node src/esp_bridge_node.cpp)

# Includes de LibSerial
target_include_directories(esp_bridge_node PRIVATE
  ${LIBSERIAL_INCLUDE_DIRS}
)

# Dependencias ROS
ament_target_dependencies(esp_bridge_node
  rclcpp
  geometry_msgs
)

# Linkeo con LibSerial
target_link_libraries(esp_bridge_node
  ${LIBSERIAL_LIBRARIES}
)

# Instalación
install(TARGETS
  esp_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
