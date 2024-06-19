cmake_minimum_required(VERSION 3.5)
project(suiryoku)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(aruku REQUIRED)
find_package(aruku_interfaces REQUIRED)
find_package(atama_interfaces REQUIRED)
find_package(kansei REQUIRED)
find_package(kansei_interfaces REQUIRED)
find_package(keisan REQUIRED)
find_package(jitsuyo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(suiryoku_interfaces REQUIRED)
find_package(tachimawari REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
include_directories(/usr/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/config/node/config_node.cpp"
  "src/${PROJECT_NAME}/locomotion/control/helper/parameter.cpp"
  "src/${PROJECT_NAME}/locomotion/control/node/control_node.cpp"
  "src/${PROJECT_NAME}/locomotion/model/robot.cpp"
  "src/${PROJECT_NAME}/locomotion/node/locomotion_node.cpp"
  "src/${PROJECT_NAME}/locomotion/process/locomotion.cpp"
  "src/${PROJECT_NAME}/node/suiryoku_node.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  aruku
  aruku_interfaces
  atama_interfaces
  kansei
  kansei_interfaces
  keisan
  jitsuyo
  rclcpp
  std_msgs
  suiryoku_interfaces
  tachimawari)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(config "src/config_main.cpp")
target_include_directories(config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(config ${PROJECT_NAME})

add_executable(locomotion "src/locomotion_main.cpp")
target_include_directories(locomotion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(locomotion ${PROJECT_NAME})

add_executable(main "src/suiryoku_main.cpp")
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(main ${PROJECT_NAME})

add_executable(forward_kinematic "src/suiryoku/node/forward_kinematic.cpp")
target_include_directories(forward_kinematic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(forward_kinematic ${PROJECT_NAME})

add_executable(forward_kinematic_subscriber "src/forward_kinematic_subscriber.cpp")
target_include_directories(forward_kinematic_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(forward_kinematic_subscriber ${PROJECT_NAME})

install(TARGETS
  config
  locomotion
  main
  forward_kinematic
  forward_kinematic_subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_index_cpp
  aruku
  aruku_interfaces
  atama_interfaces
  kansei
  kansei_interfaces
  keisan
  rclcpp
  std_msgs
  suiryoku_interfaces
  tachimawari)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
