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
CMAKE_SOURCE_DIR = /home/joe/kuka_ws/src/scripts/src/input/haption

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/joe/kuka_ws/src/scripts/src/input/haption

# Include any dependencies generated for this target.
include CMakeFiles/haptest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/haptest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/haptest.dir/flags.make

CMakeFiles/haptest.dir/haptest.cpp.o: CMakeFiles/haptest.dir/flags.make
CMakeFiles/haptest.dir/haptest.cpp.o: haptest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joe/kuka_ws/src/scripts/src/input/haption/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/haptest.dir/haptest.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/haptest.dir/haptest.cpp.o -c /home/joe/kuka_ws/src/scripts/src/input/haption/haptest.cpp

CMakeFiles/haptest.dir/haptest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/haptest.dir/haptest.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joe/kuka_ws/src/scripts/src/input/haption/haptest.cpp > CMakeFiles/haptest.dir/haptest.cpp.i

CMakeFiles/haptest.dir/haptest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/haptest.dir/haptest.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joe/kuka_ws/src/scripts/src/input/haption/haptest.cpp -o CMakeFiles/haptest.dir/haptest.cpp.s

# Object files for target haptest
haptest_OBJECTS = \
"CMakeFiles/haptest.dir/haptest.cpp.o"

# External object files for target haptest
haptest_EXTERNAL_OBJECTS =

bin/haptest: CMakeFiles/haptest.dir/haptest.cpp.o
bin/haptest: CMakeFiles/haptest.dir/build.make
bin/haptest: CMakeFiles/haptest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/joe/kuka_ws/src/scripts/src/input/haption/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/haptest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/haptest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/haptest.dir/build: bin/haptest

.PHONY : CMakeFiles/haptest.dir/build

CMakeFiles/haptest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/haptest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/haptest.dir/clean

CMakeFiles/haptest.dir/depend:
	cd /home/joe/kuka_ws/src/scripts/src/input/haption && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/joe/kuka_ws/src/scripts/src/input/haption /home/joe/kuka_ws/src/scripts/src/input/haption /home/joe/kuka_ws/src/scripts/src/input/haption /home/joe/kuka_ws/src/scripts/src/input/haption /home/joe/kuka_ws/src/scripts/src/input/haption/CMakeFiles/haptest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/haptest.dir/depend
