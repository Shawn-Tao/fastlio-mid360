cmake_minimum_required(VERSION 3.14)
project(fast_lio LANGUAGES C CXX)
if(NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(FATAL_ERROR "Source ROS 2 Humble before building this workspace.")
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_definitions(ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

find_package(ament_cmake REQUIRED)
set(dependencies
  rclcpp rclcpp_components rcl_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs
  std_srvs visualization_msgs tf2 tf2_ros pcl_ros pcl_conversions livox_ros_driver2)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED COMPONENTS CXX)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Pose6D.msg")
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(fastlio_mapping
  src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_compile_options(fastlio_mapping PRIVATE -fexceptions)
target_include_directories(fastlio_mapping PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(fastlio_mapping
  ${PCL_LIBRARIES} Python3::Python Eigen3::Eigen Threads::Threads
  "${cpp_typesupport_target}")
ament_target_dependencies(fastlio_mapping ${dependencies})

# Preserve FAST-LIO's original x86 parallelism, without conflicting -std flags.
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" fastlio_processor)
if(fastlio_processor MATCHES "^(x86_64|amd64)$" AND OpenMP_CXX_FOUND)
  include(ProcessorCount)
  ProcessorCount(fastlio_cores)
  if(fastlio_cores GREATER 4)
    target_compile_definitions(fastlio_mapping PRIVATE MP_EN MP_PROC_NUM=3)
  elseif(fastlio_cores GREATER 3)
    target_compile_definitions(fastlio_mapping PRIVATE MP_EN MP_PROC_NUM=2)
  else()
    target_compile_definitions(fastlio_mapping PRIVATE MP_PROC_NUM=1)
  endif()
else()
  target_compile_definitions(fastlio_mapping PRIVATE MP_PROC_NUM=1)
endif()
if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlio_mapping OpenMP::OpenMP_CXX)
endif()

install(TARGETS fastlio_mapping DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gt_record.py scripts/gt_postprocess.py
  DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
if(BUILD_TESTING)
  enable_testing()
  add_executable(fastlio_map_storage_test test/map_storage_test.cpp)
  target_include_directories(fastlio_map_storage_test PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(fastlio_map_storage_test ${PCL_LIBRARIES})
  add_test(NAME map_storage COMMAND fastlio_map_storage_test)
  add_test(NAME workspace_configuration COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/test_workspace.py")
  add_test(NAME workspace_tooling COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/test_tooling.py")
  add_test(NAME git_and_local_configuration COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/test_git_and_local_config.py")
endif()
ament_package()
