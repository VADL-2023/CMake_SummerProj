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
CMAKE_BINARY_DIR = /home/pi/CMake_SummerProj/MainDriver

# Include any dependencies generated for this target.
include Log/CMakeFiles/Log.dir/depend.make

# Include the progress variables for this target.
include Log/CMakeFiles/Log.dir/progress.make

# Include the compile flags for this target's objects.
include Log/CMakeFiles/Log.dir/flags.make

Log/CMakeFiles/Log.dir/src/Log.o: Log/CMakeFiles/Log.dir/flags.make
Log/CMakeFiles/Log.dir/src/Log.o: ../Log/src/Log.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/MainDriver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Log/CMakeFiles/Log.dir/src/Log.o"
	cd /home/pi/CMake_SummerProj/MainDriver/Log && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Log.dir/src/Log.o -c /home/pi/CMake_SummerProj/Log/src/Log.cpp

Log/CMakeFiles/Log.dir/src/Log.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Log.dir/src/Log.i"
	cd /home/pi/CMake_SummerProj/MainDriver/Log && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/Log/src/Log.cpp > CMakeFiles/Log.dir/src/Log.i

Log/CMakeFiles/Log.dir/src/Log.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Log.dir/src/Log.s"
	cd /home/pi/CMake_SummerProj/MainDriver/Log && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/Log/src/Log.cpp -o CMakeFiles/Log.dir/src/Log.s

# Object files for target Log
Log_OBJECTS = \
"CMakeFiles/Log.dir/src/Log.o"

# External object files for target Log
Log_EXTERNAL_OBJECTS =

Log/libLog.so: Log/CMakeFiles/Log.dir/src/Log.o
Log/libLog.so: Log/CMakeFiles/Log.dir/build.make
Log/libLog.so: VectorNav/libVN.a
Log/libLog.so: Log/CMakeFiles/Log.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/CMake_SummerProj/MainDriver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libLog.so"
	cd /home/pi/CMake_SummerProj/MainDriver/Log && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Log.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Log/CMakeFiles/Log.dir/build: Log/libLog.so

.PHONY : Log/CMakeFiles/Log.dir/build

Log/CMakeFiles/Log.dir/clean:
	cd /home/pi/CMake_SummerProj/MainDriver/Log && $(CMAKE_COMMAND) -P CMakeFiles/Log.dir/cmake_clean.cmake
.PHONY : Log/CMakeFiles/Log.dir/clean

Log/CMakeFiles/Log.dir/depend:
	cd /home/pi/CMake_SummerProj/MainDriver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/CMake_SummerProj /home/pi/CMake_SummerProj/Log /home/pi/CMake_SummerProj/MainDriver /home/pi/CMake_SummerProj/MainDriver/Log /home/pi/CMake_SummerProj/MainDriver/Log/CMakeFiles/Log.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Log/CMakeFiles/Log.dir/depend

