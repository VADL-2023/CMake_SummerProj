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
include pigpio-master/CMakeFiles/pigs.dir/depend.make

# Include the progress variables for this target.
include pigpio-master/CMakeFiles/pigs.dir/progress.make

# Include the compile flags for this target's objects.
include pigpio-master/CMakeFiles/pigs.dir/flags.make

pigpio-master/CMakeFiles/pigs.dir/pigs.c.o: pigpio-master/CMakeFiles/pigs.dir/flags.make
pigpio-master/CMakeFiles/pigs.dir/pigs.c.o: ../pigpio-master/pigs.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vadl/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object pigpio-master/CMakeFiles/pigs.dir/pigs.c.o"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pigs.dir/pigs.c.o -c /home/vadl/CMake_SummerProj/pigpio-master/pigs.c

pigpio-master/CMakeFiles/pigs.dir/pigs.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pigs.dir/pigs.c.i"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/vadl/CMake_SummerProj/pigpio-master/pigs.c > CMakeFiles/pigs.dir/pigs.c.i

pigpio-master/CMakeFiles/pigs.dir/pigs.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pigs.dir/pigs.c.s"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/vadl/CMake_SummerProj/pigpio-master/pigs.c -o CMakeFiles/pigs.dir/pigs.c.s

pigpio-master/CMakeFiles/pigs.dir/command.c.o: pigpio-master/CMakeFiles/pigs.dir/flags.make
pigpio-master/CMakeFiles/pigs.dir/command.c.o: ../pigpio-master/command.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vadl/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object pigpio-master/CMakeFiles/pigs.dir/command.c.o"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pigs.dir/command.c.o -c /home/vadl/CMake_SummerProj/pigpio-master/command.c

pigpio-master/CMakeFiles/pigs.dir/command.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pigs.dir/command.c.i"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/vadl/CMake_SummerProj/pigpio-master/command.c > CMakeFiles/pigs.dir/command.c.i

pigpio-master/CMakeFiles/pigs.dir/command.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pigs.dir/command.c.s"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/vadl/CMake_SummerProj/pigpio-master/command.c -o CMakeFiles/pigs.dir/command.c.s

# Object files for target pigs
pigs_OBJECTS = \
"CMakeFiles/pigs.dir/pigs.c.o" \
"CMakeFiles/pigs.dir/command.c.o"

# External object files for target pigs
pigs_EXTERNAL_OBJECTS =

pigpio-master/pigs: pigpio-master/CMakeFiles/pigs.dir/pigs.c.o
pigpio-master/pigs: pigpio-master/CMakeFiles/pigs.dir/command.c.o
pigpio-master/pigs: pigpio-master/CMakeFiles/pigs.dir/build.make
pigpio-master/pigs: pigpio-master/CMakeFiles/pigs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vadl/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable pigs"
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pigs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pigpio-master/CMakeFiles/pigs.dir/build: pigpio-master/pigs

.PHONY : pigpio-master/CMakeFiles/pigs.dir/build

pigpio-master/CMakeFiles/pigs.dir/clean:
	cd /home/vadl/CMake_SummerProj/build/pigpio-master && $(CMAKE_COMMAND) -P CMakeFiles/pigs.dir/cmake_clean.cmake
.PHONY : pigpio-master/CMakeFiles/pigs.dir/clean

pigpio-master/CMakeFiles/pigs.dir/depend:
	cd /home/vadl/CMake_SummerProj/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vadl/CMake_SummerProj /home/vadl/CMake_SummerProj/pigpio-master /home/vadl/CMake_SummerProj/build /home/vadl/CMake_SummerProj/build/pigpio-master /home/vadl/CMake_SummerProj/build/pigpio-master/CMakeFiles/pigs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pigpio-master/CMakeFiles/pigs.dir/depend

