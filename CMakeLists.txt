cmake_minimum_required(VERSION 3.5)
project(ros2_rknn_llm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nlohmann_json_schema_validator_vendor REQUIRED)

set(RKLLM_ROOT "")
if(DEFINED ENV{RKLLM_SDK_DIR})
  set(RKLLM_ROOT $ENV{RKLLM_SDK_DIR})
elseif(DEFINED ENV{RKLLM_ROOT})
  set(RKLLM_ROOT $ENV{RKLLM_ROOT})
endif()

if(RKLLM_ROOT STREQUAL "")
  message(FATAL_ERROR "RKLLM SDK not found. Set RKLLM_SDK_DIR or RKLLM_ROOT to the SDK path.")
endif()

find_path(RKLLM_INCLUDE_DIR rkllm.h
  HINTS ${RKLLM_ROOT} ${RKLLM_ROOT}/include
)
find_library(RKLLM_LIBRARY NAMES rkllmrt rkllm
  HINTS
    ${RKLLM_ROOT}
    ${RKLLM_ROOT}/lib
    ${RKLLM_ROOT}/lib64
    ${RKLLM_ROOT}/aarch64
    ${RKLLM_ROOT}/armhf
)

if(NOT RKLLM_INCLUDE_DIR OR NOT RKLLM_LIBRARY)
  message(FATAL_ERROR "RKLLM SDK not found. Expected rkllm.h and librkllmrt.so/librkllm.so under ${RKLLM_ROOT}.")
endif()

add_library(ros2_rknn_llm_core
  src/rkllm_engine.cpp
  src/planner_client.cpp
)

ament_target_dependencies(ros2_rknn_llm_core
  rclcpp
  std_msgs
  std_srvs
  ament_index_cpp
  nlohmann_json
  nlohmann_json_schema_validator_vendor
)

target_include_directories(ros2_rknn_llm_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${RKLLM_INCLUDE_DIR}
)

target_link_libraries(ros2_rknn_llm_core
  ${RKLLM_LIBRARY}
  dl
)

add_executable(rkllm_node
  src/ros2_rknn_llm_node.cpp
)

ament_target_dependencies(rkllm_node
  rclcpp
  std_msgs
  std_srvs
  ament_index_cpp
  nlohmann_json
  nlohmann_json_schema_validator_vendor
)

target_link_libraries(rkllm_node
  ros2_rknn_llm_core
)

install(TARGETS
  ros2_rknn_llm_core
  rkllm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


ament_package()
