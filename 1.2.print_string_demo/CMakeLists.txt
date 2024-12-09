cmake_minimum_required(VERSION 3.16.3)
project(math)


set(SOURCE
	./main.cpp
)

message(STATUS ${PROJECT_NAME})
add_executable(${PROJECT_NAME} ${SOURCE})

target_include_directories(${PROJECT_NAME}
	PRIVATE
	${CMAKE_SOURCE_DIR}
)



