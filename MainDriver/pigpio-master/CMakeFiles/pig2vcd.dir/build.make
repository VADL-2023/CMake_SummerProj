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
include pigpio-master/CMakeFiles/pig2vcd.dir/depend.make

# Include the progress variables for this target.
include pigpio-master/CMakeFiles/pig2vcd.dir/progress.make

# Include the compile flags for this target's objects.
include pigpio-master/CMakeFiles/pig2vcd.dir/flags.make

pigpio-master/CMakeFiles/pig2vcd.dir/pig2vcd.c.o: pigpio-master/CMakeFiles/pig2vcd.dir/flags.make
pigpio-master/CMakeFiles/pig2vcd.dir/pig2vcd.c.o: ../pigpio-master/pig2vcd.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/MainDriver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object pigpio-master/CMakeFiles/pig2vcd.dir/pig2vcd.c.o"
	cd /home/pi/CMake_SummerProj/MainDriver/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pig2vcd.dir/pig2vcd.c.o -c /home/pi/CMake_SummerProj/pigpio-master/pig2vcd.c

pigpio-master/CMakeFiles/pig2vcd.dir/pig2vcd.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pig2vcd.dir/pig2vcd.c.i"
	cd /home/pi/CMake_SummerProj/MainDriver/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/CMake_SummerProj/pigpio-master/pig2vcd.c > CMakeFiles/pig2vcd.dir/pig2vcd.c.i

pigpio-master/CMakeFiles/pig2vcd.dir/pig2vcd.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pig2vcd.dir/pig2vcd.c.s"
	cd /home/pi/CMake_SummerProj/MainDriver/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/CMake_SummerProj/pigpio-master/pig2vcd.c -o CMakeFiles/pig2vcd.dir/pig2vcd.c.s

pigpio-master/CMakeFiles/pig2vcd.dir/command.c.o: pigpio-master/CMakeFiles/pig2vcd.dir/flags.make
pigpio-master/CMakeFiles/pig2vcd.dir/command.c.o: ../pigpio-master/command.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/MainDriver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object pigpio-master/CMakeFiles/pig2vcd.dir/command.c.o"
	cd /home/pi/CMake_SummerProj/MainDriver/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pig2vcd.dir/command.c.o -c /home/pi/CMake_SummerProj/pigpio-master/command.c

pigpio-master/CMakeFiles/pig2vcd.dir/command.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pig2vcd.dir/command.c.i"
	cd /home/pi/CMake_SummerProj/MainDriver/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/CMake_SummerProj/pigpio-master/command.c > CMakeFiles/pig2vcd.dir/command.c.i

pigpio-master/CMakeFiles/pig2vcd.dir/command.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pig2vcd.dir/command.c.s"
	cd /home/pi/CMake_SummerProj/MainDriver/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/CMake_SummerProj/pigpio-master/command.c -o CMakeFiles/pig2vcd.dir/command.c.s

# Object files for target pig2vcd
pig2vcd_OBJECTS = \
"CMakeFiles/pig2vcd.dir/pig2vcd.c.o" \
"CMakeFiles/pig2vcd.dir/command.c.o"

# External object files for target pig2vcd
pig2vcd_EXTERNAL_OBJECTS =

pigpio-master/pig2vcd: pigpio-master/CMakeFiles/pig2vcd.dir/pig2vcd.c.o
pigpio-master/pig2vcd: pigpio-master/CMakeFiles/pig2vcd.dir/command.c.o
pigpio-master/pig2vcd: pigpio-master/CMakeFiles/pig2vcd.dir/build.make
pigpio-master/pig2vcd: pigpio-master/CMakeFiles/pig2vcd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/CMake_SummerProj/MainDriver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable pig2vcd"
	cd /home/pi/CMake_SummerProj/MainDriver/pigpio-master && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pig2vcd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pigpio-master/CMakeFiles/pig2vcd.dir/build: pigpio-master/pig2vcd

.PHONY : pigpio-master/CMakeFiles/pig2vcd.dir/build

pigpio-master/CMakeFiles/pig2vcd.dir/clean:
	cd /home/pi/CMake_SummerProj/MainDriver/pigpio-master && $(CMAKE_COMMAND) -P CMakeFiles/pig2vcd.dir/cmake_clean.cmake
.PHONY : pigpio-master/CMakeFiles/pig2vcd.dir/clean

pigpio-master/CMakeFiles/pig2vcd.dir/depend:
	cd /home/pi/CMake_SummerProj/MainDriver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/CMake_SummerProj /home/pi/CMake_SummerProj/pigpio-master /home/pi/CMake_SummerProj/MainDriver /home/pi/CMake_SummerProj/MainDriver/pigpio-master /home/pi/CMake_SummerProj/MainDriver/pigpio-master/CMakeFiles/pig2vcd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pigpio-master/CMakeFiles/pig2vcd.dir/depend

