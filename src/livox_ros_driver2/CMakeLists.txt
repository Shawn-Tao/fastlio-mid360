cmake_minimum_required(VERSION 3.14)
include(cmake/version.cmake)
project(livox_ros_driver2 VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES C CXX)

if(NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(FATAL_ERROR "This workspace supports ROS 2 Humble only. Source /opt/ros/humble/setup.bash (or setup.zsh) first.")
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_definitions(BUILDING_ROS2)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(PCL REQUIRED COMPONENTS common io)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(APR REQUIRED apr-1)
find_package(Threads REQUIRED)
include(cmake/livox_sdk.cmake)

# A separate interface target avoids the ament Python/library target collision.
set(LIVOX_INTERFACES livox_interfaces2)
rosidl_generate_interfaces(${LIVOX_INTERFACES}
  "msg/CustomPoint.msg" "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME})

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/livox_ros_driver2.cpp src/lddc.cpp src/driver_node.cpp
  src/lds.cpp src/lds_lidar.cpp
  src/comm/comm.cpp src/comm/ldq.cpp src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp src/comm/cache_index.cpp src/comm/pub_handler.cpp
  src/parse_cfg_file/parse_cfg_file.cpp src/parse_cfg_file/parse_livox_lidar_cfg.cpp
  src/call_back/lidar_common_callback.cpp src/call_back/livox_lidar_callback.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${PCL_INCLUDE_DIRS} ${APR_INCLUDE_DIRS} ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
  "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty" "${CMAKE_CURRENT_SOURCE_DIR}/src")
rosidl_get_typesupport_target(cpp_typesupport_target
  ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}
  "${cpp_typesupport_target}" "${LIVOX_LIDAR_SDK_LIBRARY}"
  ${PCL_LIBRARIES} ${APR_LIBRARIES} Threads::Threads)

# Install the selected SDK alongside the driver, no global SDK installation
# and no manually maintained LD_LIBRARY_PATH needed after source install/.
if(LIVOX_LIDAR_SDK_LIBRARY MATCHES "\\.so$")
  install(FILES "${LIVOX_LIDAR_SDK_LIBRARY}" DESTINATION lib)
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livox_ros::DriverNode" EXECUTABLE ${PROJECT_NAME}_node)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  enable_testing()
  foreach(arch x86_64 amd64 aarch64 arm64)
    add_test(NAME sdk_${arch} COMMAND ${CMAKE_COMMAND}
      -DTEST_PROCESSOR=${arch} -DPROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}
      -P ${PROJECT_SOURCE_DIR}/test/sdk_selection.cmake)
  endforeach()
  foreach(arch armv7l i686 riscv64)
    add_test(NAME sdk_reject_${arch} COMMAND ${CMAKE_COMMAND}
      -DTEST_PROCESSOR=${arch} -DPROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}
      -P ${PROJECT_SOURCE_DIR}/test/sdk_selection.cmake)
    set_tests_properties(sdk_reject_${arch} PROPERTIES WILL_FAIL TRUE)
  endforeach()
  add_test(NAME sdk_reject_wrong_override COMMAND ${CMAKE_COMMAND}
    -DTEST_PROCESSOR=x86_64 -DPROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}
    -DLIVOX_SDK_ROOT=${PROJECT_SOURCE_DIR}/../../livox-sdk-arm
    -P ${PROJECT_SOURCE_DIR}/test/sdk_selection.cmake)
  set_tests_properties(sdk_reject_wrong_override PROPERTIES WILL_FAIL TRUE)
endif()
ament_auto_package(INSTALL_TO_SHARE config launch_ROS2)
