cmake_minimum_required(VERSION 3.8)
project(rov_sediment)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the action interface.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CollectSediment.action"
  DEPENDENCIES action_msgs
)
# Let same-package C++ targets link the generated typesupport.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Lifecycle node + action server.
add_executable(sediment_collector_node src/sediment_collector_node.cpp)
ament_target_dependencies(sediment_collector_node
  rclcpp rclcpp_action rclcpp_lifecycle lifecycle_msgs std_msgs)
target_link_libraries(sediment_collector_node "${cpp_typesupport_target}")

# Action client (demo / test driver).
add_executable(collect_client src/collect_client.cpp)
ament_target_dependencies(collect_client rclcpp rclcpp_action)
target_link_libraries(collect_client "${cpp_typesupport_target}")

install(TARGETS sediment_collector_node collect_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
