cmake_minimum_required(VERSION 3.5)
project(target-tracking)

set(CMAKE_CXX_STANDARD 14)

message( "\n${PROJECT_NAME}...")

# CUDA
find_package(CUDA REQUIRED)
message(STATUS "Find CUDA include at ${CUDA_INCLUDE_DIRS}")
message(STATUS "Find CUDA libraries: ${CUDA_LIBRARIES}")

# TensorRT
set(TENSORRT_ROOT "/home/chaucer/TensorRT-8.0.1.6")
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
        HINTS ${TENSORRT_ROOT} PATH_SUFFIXES include/)
message(STATUS "Found TensorRT headers at ${TENSORRT_INCLUDE_DIR}")
find_library(TENSORRT_LIBRARY_INFER nvinfer
        HINTS ${TENSORRT_ROOT} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)
find_library(TENSORRT_LIBRARY_ONNXPARSER nvonnxparser
        HINTS  ${TENSORRT_ROOT} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)
set(TENSORRT_LIBRARY ${TENSORRT_LIBRARY_INFER} ${TENSORRT_LIBRARY_ONNXPARSER})
message(STATUS "Find TensorRT libs: ${TENSORRT_LIBRARY}")

# OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "Find OpenCV include at ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Find OpenCV libraries: ${OpenCV_LIBRARIES}")
add_definitions(-DOPEN_CV_INSTALLED)

set(COMMON_INCLUDE ./includes/common)

# YAML
find_package(yaml-cpp REQUIRED)
message(STATUS "Find YAML include at ${YAML_CPP_INCLUDE_DIR}")
message(STATUS "Find YAMLs include at ${YAML_INCLUDE_DIRS}")

# json
find_package(jsoncpp REQUIRED)
set(JSONCPP_INCLUDE_DIRS "/usr/include/jsoncpp")
message(STATUS "Find json include at ${JSONCPP_INCLUDE_DIR}")

set(HELPER_FUNCTIONS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)


include_directories(${CUDA_INCLUDE_DIRS} ${TENSORRT_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${COMMON_INCLUDE} ${YAML_INCLUDE_DIRS})
include_directories(${JSONCPP_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
  ${SOURCE_FILES}
  ${HELPER_FUNCTIONS_DIR}/dji_linux_environment.cpp
  ${HELPER_FUNCTIONS_DIR}/dji_linux_helpers.cpp

  target-tracking.cpp
  )
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} pthread ${OpenCV_LIBRARIES} ${CUDA_LIBRARIES} ${TENSORRT_LIBRARY} yaml-cpp jsoncpp)
