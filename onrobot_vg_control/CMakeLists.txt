cmake_minimum_required(VERSION 3.16)
project(onrobot_vg_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(onrobot_vg_modbus_tcp)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  nodes/OnRobotVGSimpleController.py
  nodes/OnRobotVGStatusListener.py
  nodes/OnRobotVGTcpNode.py
  nodes/OnRobotVGSimpleControllerServer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_vg_failed_read test/test_vg_failed_read.py)
endif()

ament_package()

