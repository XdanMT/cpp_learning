cmake_minimum_required(VERSION 3.16.3)
# project(cloud_compute_cuda LANGUAGES CXX CUDA)
project(eigen_pcl_test)

# Add CUDA files to the project
set(SOURCE
	./main.cpp
)

# add executable
ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCE})


set(ALL_TARGET_LIBRARIES "")

# add .cmake
include(cmake/eigen.cmake)
include(cmake/PCL.cmake)

# link party3rd
message(STATUS "ALL_TARGET_LIBRARIES: ${ALL_TARGET_LIBRARIES}")
target_link_libraries(${PROJECT_NAME} PRIVATE
	${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES}
)

