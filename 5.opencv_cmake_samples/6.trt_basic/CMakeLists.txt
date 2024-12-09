# 最低版本要求
cmake_minimum_required(VERSION 3.10)

# 项目信息
project(trt_demo LANGUAGES CXX CUDA)

# 添加CMAKE_MODULE_PATH，否则找不到FindTensorRT.cmake
list (APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# 寻找TensorRT库
find_package(TensorRT REQUIRED)

if (TensorRT_FOUND)
    message(STATUS "Found TensorRT ${TensorRT_VERSION} in ${TensorRT_ROOT_DIR}")
    message(STATUS "TensorRT libraries: ${TensorRT_LIBRARIES}")
    message(STATUS "TensorRT include files: ${TensorRT_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "Cannot find TensorRT")

endif()

# 寻找CUDA
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
set(CUDA_LIBRARIES "${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcudart.so")
 
# 添加可执行文件
add_executable(build src/build.cpp)

# 头文件
target_include_directories(build PRIVATE ${TensorRT_INCLUDE_DIRS})
target_include_directories(build PRIVATE ${CUDA_INCLUDE_DIRS})
# 链接库
target_link_libraries(build PRIVATE ${TensorRT_LIBRARIES})
target_link_libraries(build PRIVATE ${CUDA_LIBRARIES})


# 添加可执行文件
add_executable(runtime src/runtime.cu)

# 头文件
target_include_directories(runtime PRIVATE ${TensorRT_INCLUDE_DIRS})
# 链接库
target_link_libraries(runtime PRIVATE ${TensorRT_LIBRARIES})

