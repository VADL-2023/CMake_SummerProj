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
include pigpio-master/CMakeFiles/pigpiod_if2.dir/depend.make

# Include the progress variables for this target.
include pigpio-master/CMakeFiles/pigpiod_if2.dir/progress.make

# Include the compile flags for this target's objects.
include pigpio-master/CMakeFiles/pigpiod_if2.dir/flags.make

pigpio-master/CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.o: pigpio-master/CMakeFiles/pigpiod_if2.dir/flags.make
pigpio-master/CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.o: ../pigpio-master/pigpiod_if2.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object pigpio-master/CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.o"
	cd /home/pi/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.o -c /home/pi/CMake_SummerProj/pigpio-master/pigpiod_if2.c

pigpio-master/CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.i"
	cd /home/pi/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/CMake_SummerProj/pigpio-master/pigpiod_if2.c > CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.i

pigpio-master/CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.s"
	cd /home/pi/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/CMake_SummerProj/pigpio-master/pigpiod_if2.c -o CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.s

pigpio-master/CMakeFiles/pigpiod_if2.dir/command.c.o: pigpio-master/CMakeFiles/pigpiod_if2.dir/flags.make
pigpio-master/CMakeFiles/pigpiod_if2.dir/command.c.o: ../pigpio-master/command.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object pigpio-master/CMakeFiles/pigpiod_if2.dir/command.c.o"
	cd /home/pi/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pigpiod_if2.dir/command.c.o -c /home/pi/CMake_SummerProj/pigpio-master/command.c

pigpio-master/CMakeFiles/pigpiod_if2.dir/command.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pigpiod_if2.dir/command.c.i"
	cd /home/pi/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/CMake_SummerProj/pigpio-master/command.c > CMakeFiles/pigpiod_if2.dir/command.c.i

pigpio-master/CMakeFiles/pigpiod_if2.dir/command.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pigpiod_if2.dir/command.c.s"
	cd /home/pi/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/CMake_SummerProj/pigpio-master/command.c -o CMakeFiles/pigpiod_if2.dir/command.c.s

# Object files for target pigpiod_if2
pigpiod_if2_OBJECTS = \
"CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.o" \
"CMakeFiles/pigpiod_if2.dir/command.c.o"

# External object files for target pigpiod_if2
pigpiod_if2_EXTERNAL_OBJECTS =

pigpio-master/libpigpiod_if2.so: pigpio-master/CMakeFiles/pigpiod_if2.dir/pigpiod_if2.c.o
pigpio-master/libpigpiod_if2.so: pigpio-master/CMakeFiles/pigpiod_if2.dir/command.c.o
pigpio-master/libpigpiod_if2.so: pigpio-master/CMakeFiles/pigpiod_if2.dir/build.make
pigpio-master/libpigpiod_if2.so: pigpio-master/CMakeFiles/pigpiod_if2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C shared library libpigpiod_if2.so"
	cd /home/pi/CMake_SummerProj/build/pigpio-master && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pigpiod_if2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pigpio-master/CMakeFiles/pigpiod_if2.dir/build: pigpio-master/libpigpiod_if2.so

.PHONY : pigpio-master/CMakeFiles/pigpiod_if2.dir/build

pigpio-master/CMakeFiles/pigpiod_if2.dir/clean:
	cd /home/pi/CMake_SummerProj/build/pigpio-master && $(CMAKE_COMMAND) -P CMakeFiles/pigpiod_if2.dir/cmake_clean.cmake
.PHONY : pigpio-master/CMakeFiles/pigpiod_if2.dir/clean

pigpio-master/CMakeFiles/pigpiod_if2.dir/depend:
	cd /home/pi/CMake_SummerProj/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/CMake_SummerProj /home/pi/CMake_SummerProj/pigpio-master /home/pi/CMake_SummerProj/build /home/pi/CMake_SummerProj/build/pigpio-master /home/pi/CMake_SummerProj/build/pigpio-master/CMakeFiles/pigpiod_if2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pigpio-master/CMakeFiles/pigpiod_if2.dir/depend
