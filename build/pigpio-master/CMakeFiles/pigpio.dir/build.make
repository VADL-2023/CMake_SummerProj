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
CMAKE_SOURCE_DIR = /home/vadl/CMake_SummerProj

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vadl/CMake_SummerProj/build

# Include any dependencies generated for this target.
include pigpio-master/CMakeFiles/pigpio.dir/depend.make

# Include the progress variables for this target.
include pigpio-master/CMakeFiles/pigpio.dir/progress.make

# Include the compile flags for this target's objects.
include pigpio-master/CMakeFiles/pigpio.dir/flags.make

pigpio-master/CMakeFiles/pigpio.dir/pigpio.c.o: pigpio-master/CMakeFiles/pigpio.dir/flags.make
pigpio-master/CMakeFiles/pigpio.dir/pigpio.c.o: ../pigpio-master/pigpio.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vadl/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object pigpio-master/CMakeFiles/pigpio.dir/pigpio.c.o"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pigpio.dir/pigpio.c.o -c /home/vadl/CMake_SummerProj/pigpio-master/pigpio.c

pigpio-master/CMakeFiles/pigpio.dir/pigpio.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pigpio.dir/pigpio.c.i"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/vadl/CMake_SummerProj/pigpio-master/pigpio.c > CMakeFiles/pigpio.dir/pigpio.c.i

pigpio-master/CMakeFiles/pigpio.dir/pigpio.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pigpio.dir/pigpio.c.s"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/vadl/CMake_SummerProj/pigpio-master/pigpio.c -o CMakeFiles/pigpio.dir/pigpio.c.s

pigpio-master/CMakeFiles/pigpio.dir/command.c.o: pigpio-master/CMakeFiles/pigpio.dir/flags.make
pigpio-master/CMakeFiles/pigpio.dir/command.c.o: ../pigpio-master/command.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vadl/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object pigpio-master/CMakeFiles/pigpio.dir/command.c.o"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pigpio.dir/command.c.o -c /home/vadl/CMake_SummerProj/pigpio-master/command.c

pigpio-master/CMakeFiles/pigpio.dir/command.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pigpio.dir/command.c.i"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/vadl/CMake_SummerProj/pigpio-master/command.c > CMakeFiles/pigpio.dir/command.c.i

pigpio-master/CMakeFiles/pigpio.dir/command.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pigpio.dir/command.c.s"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/vadl/CMake_SummerProj/pigpio-master/command.c -o CMakeFiles/pigpio.dir/command.c.s

# Object files for target pigpio
pigpio_OBJECTS = \
"CMakeFiles/pigpio.dir/pigpio.c.o" \
"CMakeFiles/pigpio.dir/command.c.o"

# External object files for target pigpio
pigpio_EXTERNAL_OBJECTS =

pigpio-master/libpigpio.so: pigpio-master/CMakeFiles/pigpio.dir/pigpio.c.o
pigpio-master/libpigpio.so: pigpio-master/CMakeFiles/pigpio.dir/command.c.o
pigpio-master/libpigpio.so: pigpio-master/CMakeFiles/pigpio.dir/build.make
pigpio-master/libpigpio.so: pigpio-master/CMakeFiles/pigpio.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vadl/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C shared library libpigpio.so"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pigpio.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pigpio-master/CMakeFiles/pigpio.dir/build: pigpio-master/libpigpio.so

.PHONY : pigpio-master/CMakeFiles/pigpio.dir/build

pigpio-master/CMakeFiles/pigpio.dir/clean:
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && $(CMAKE_COMMAND) -P CMakeFiles/pigpio.dir/cmake_clean.cmake
.PHONY : pigpio-master/CMakeFiles/pigpio.dir/clean

pigpio-master/CMakeFiles/pigpio.dir/depend:
	cd /home/vadl/CMake_SummerProj/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vadl/CMake_SummerProj /home/vadl/CMake_SummerProj/pigpio-master /home/vadl/CMake_SummerProj/build /home/vadl/CMake_SummerProj/build/pigpio-master /home/vadl/CMake_SummerProj/build/pigpio-master/CMakeFiles/pigpio.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pigpio-master/CMakeFiles/pigpio.dir/depend

