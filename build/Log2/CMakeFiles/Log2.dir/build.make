# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/CMake_SummerProj

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/CMake_SummerProj/build

# Include any dependencies generated for this target.
include Log2/CMakeFiles/Log2.dir/depend.make

# Include the progress variables for this target.
include Log2/CMakeFiles/Log2.dir/progress.make

# Include the compile flags for this target's objects.
include Log2/CMakeFiles/Log2.dir/flags.make

Log2/CMakeFiles/Log2.dir/src/Log2.o: Log2/CMakeFiles/Log2.dir/flags.make
Log2/CMakeFiles/Log2.dir/src/Log2.o: ../Log2/src/Log2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Log2/CMakeFiles/Log2.dir/src/Log2.o"
	cd /home/pi/CMake_SummerProj/build/Log2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Log2.dir/src/Log2.o -c /home/pi/CMake_SummerProj/Log2/src/Log2.cpp

Log2/CMakeFiles/Log2.dir/src/Log2.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Log2.dir/src/Log2.i"
	cd /home/pi/CMake_SummerProj/build/Log2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/Log2/src/Log2.cpp > CMakeFiles/Log2.dir/src/Log2.i

Log2/CMakeFiles/Log2.dir/src/Log2.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Log2.dir/src/Log2.s"
	cd /home/pi/CMake_SummerProj/build/Log2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/Log2/src/Log2.cpp -o CMakeFiles/Log2.dir/src/Log2.s

# Object files for target Log2
Log2_OBJECTS = \
"CMakeFiles/Log2.dir/src/Log2.o"

# External object files for target Log2
Log2_EXTERNAL_OBJECTS =

Log2/libLog2.so: Log2/CMakeFiles/Log2.dir/src/Log2.o
Log2/libLog2.so: Log2/CMakeFiles/Log2.dir/build.make
Log2/libLog2.so: VectorNav/libVN.so
Log2/libLog2.so: Log2/CMakeFiles/Log2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libLog2.so"
	cd /home/pi/CMake_SummerProj/build/Log2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Log2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Log2/CMakeFiles/Log2.dir/build: Log2/libLog2.so

.PHONY : Log2/CMakeFiles/Log2.dir/build

Log2/CMakeFiles/Log2.dir/clean:
	cd /home/pi/CMake_SummerProj/build/Log2 && $(CMAKE_COMMAND) -P CMakeFiles/Log2.dir/cmake_clean.cmake
.PHONY : Log2/CMakeFiles/Log2.dir/clean

Log2/CMakeFiles/Log2.dir/depend:
	cd /home/pi/CMake_SummerProj/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/CMake_SummerProj /home/pi/CMake_SummerProj/Log2 /home/pi/CMake_SummerProj/build /home/pi/CMake_SummerProj/build/Log2 /home/pi/CMake_SummerProj/build/Log2/CMakeFiles/Log2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Log2/CMakeFiles/Log2.dir/depend

