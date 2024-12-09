cmake_minimum_required(VERSION 3.10)
project(MyProject)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define the dynamic library
add_library(my_library SHARED
    src/my_library.cpp
)

# Export the dynamic library API
target_include_directories(my_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Define the main executable
add_executable(main_executable
    src/main.cpp
)

# Link the dynamic library to the main executable
target_link_libraries(main_executable  my_library)

