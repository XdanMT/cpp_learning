# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wzj/wzj/repos/cpp_test/opencv_cmake_samples/6.trt_basic

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wzj/wzj/repos/cpp_test/opencv_cmake_samples/6.trt_basic/build

# Include any dependencies generated for this target.
include CMakeFiles/runtime.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/runtime.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/runtime.dir/flags.make

CMakeFiles/runtime.dir/src/runtime.cu.o: CMakeFiles/runtime.dir/flags.make
CMakeFiles/runtime.dir/src/runtime.cu.o: ../src/runtime.cu
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wzj/wzj/repos/cpp_test/opencv_cmake_samples/6.trt_basic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CUDA object CMakeFiles/runtime.dir/src/runtime.cu.o"
	/usr/local/cuda-11.6/bin/nvcc  $(CUDA_DEFINES) $(CUDA_INCLUDES) $(CUDA_FLAGS) -x cu -c /home/wzj/wzj/repos/cpp_test/opencv_cmake_samples/6.trt_basic/src/runtime.cu -o CMakeFiles/runtime.dir/src/runtime.cu.o

CMakeFiles/runtime.dir/src/runtime.cu.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CUDA source to CMakeFiles/runtime.dir/src/runtime.cu.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_PREPROCESSED_SOURCE

CMakeFiles/runtime.dir/src/runtime.cu.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CUDA source to assembly CMakeFiles/runtime.dir/src/runtime.cu.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_ASSEMBLY_SOURCE

# Object files for target runtime
runtime_OBJECTS = \
"CMakeFiles/runtime.dir/src/runtime.cu.o"

# External object files for target runtime
runtime_EXTERNAL_OBJECTS =

runtime: CMakeFiles/runtime.dir/src/runtime.cu.o
runtime: CMakeFiles/runtime.dir/build.make
runtime: /usr/lib/x86_64-linux-gnu/libnvinfer.so
runtime: /usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so
runtime: /usr/lib/x86_64-linux-gnu/libnvparsers.so
runtime: /usr/lib/x86_64-linux-gnu/libnvonnxparser.so
runtime: CMakeFiles/runtime.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wzj/wzj/repos/cpp_test/opencv_cmake_samples/6.trt_basic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CUDA executable runtime"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/runtime.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/runtime.dir/build: runtime

.PHONY : CMakeFiles/runtime.dir/build

CMakeFiles/runtime.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/runtime.dir/cmake_clean.cmake
.PHONY : CMakeFiles/runtime.dir/clean

CMakeFiles/runtime.dir/depend:
	cd /home/wzj/wzj/repos/cpp_test/opencv_cmake_samples/6.trt_basic/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wzj/wzj/repos/cpp_test/opencv_cmake_samples/6.trt_basic /home/wzj/wzj/repos/cpp_test/opencv_cmake_samples/6.trt_basic /home/wzj/wzj/repos/cpp_test/opencv_cmake_samples/6.trt_basic/build /home/wzj/wzj/repos/cpp_test/opencv_cmake_samples/6.trt_basic/build /home/wzj/wzj/repos/cpp_test/opencv_cmake_samples/6.trt_basic/build/CMakeFiles/runtime.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/runtime.dir/depend
