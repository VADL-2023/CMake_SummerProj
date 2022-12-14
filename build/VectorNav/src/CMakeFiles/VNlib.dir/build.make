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
CMAKE_SOURCE_DIR = /home/pi/CMakeListsTest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/CMakeListsTest/build

# Include any dependencies generated for this target.
include VectorNav/src/CMakeFiles/VNlib.dir/depend.make

# Include the progress variables for this target.
include VectorNav/src/CMakeFiles/VNlib.dir/progress.make

# Include the compile flags for this target's objects.
include VectorNav/src/CMakeFiles/VNlib.dir/flags.make

VectorNav/src/CMakeFiles/VNlib.dir/attitude.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/attitude.o: ../VectorNav/src/attitude.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/attitude.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/attitude.o -c /home/pi/CMakeListsTest/VectorNav/src/attitude.cpp

VectorNav/src/CMakeFiles/VNlib.dir/attitude.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/attitude.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/attitude.cpp > CMakeFiles/VNlib.dir/attitude.i

VectorNav/src/CMakeFiles/VNlib.dir/attitude.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/attitude.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/attitude.cpp -o CMakeFiles/VNlib.dir/attitude.s

VectorNav/src/CMakeFiles/VNlib.dir/compositedata.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/compositedata.o: ../VectorNav/src/compositedata.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/compositedata.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/compositedata.o -c /home/pi/CMakeListsTest/VectorNav/src/compositedata.cpp

VectorNav/src/CMakeFiles/VNlib.dir/compositedata.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/compositedata.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/compositedata.cpp > CMakeFiles/VNlib.dir/compositedata.i

VectorNav/src/CMakeFiles/VNlib.dir/compositedata.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/compositedata.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/compositedata.cpp -o CMakeFiles/VNlib.dir/compositedata.s

VectorNav/src/CMakeFiles/VNlib.dir/conversions.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/conversions.o: ../VectorNav/src/conversions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/conversions.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/conversions.o -c /home/pi/CMakeListsTest/VectorNav/src/conversions.cpp

VectorNav/src/CMakeFiles/VNlib.dir/conversions.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/conversions.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/conversions.cpp > CMakeFiles/VNlib.dir/conversions.i

VectorNav/src/CMakeFiles/VNlib.dir/conversions.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/conversions.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/conversions.cpp -o CMakeFiles/VNlib.dir/conversions.s

VectorNav/src/CMakeFiles/VNlib.dir/criticalsection.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/criticalsection.o: ../VectorNav/src/criticalsection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/criticalsection.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/criticalsection.o -c /home/pi/CMakeListsTest/VectorNav/src/criticalsection.cpp

VectorNav/src/CMakeFiles/VNlib.dir/criticalsection.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/criticalsection.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/criticalsection.cpp > CMakeFiles/VNlib.dir/criticalsection.i

VectorNav/src/CMakeFiles/VNlib.dir/criticalsection.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/criticalsection.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/criticalsection.cpp -o CMakeFiles/VNlib.dir/criticalsection.s

VectorNav/src/CMakeFiles/VNlib.dir/dllvalidator.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/dllvalidator.o: ../VectorNav/src/dllvalidator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/dllvalidator.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/dllvalidator.o -c /home/pi/CMakeListsTest/VectorNav/src/dllvalidator.cpp

VectorNav/src/CMakeFiles/VNlib.dir/dllvalidator.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/dllvalidator.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/dllvalidator.cpp > CMakeFiles/VNlib.dir/dllvalidator.i

VectorNav/src/CMakeFiles/VNlib.dir/dllvalidator.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/dllvalidator.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/dllvalidator.cpp -o CMakeFiles/VNlib.dir/dllvalidator.s

VectorNav/src/CMakeFiles/VNlib.dir/event.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/event.o: ../VectorNav/src/event.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/event.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/event.o -c /home/pi/CMakeListsTest/VectorNav/src/event.cpp

VectorNav/src/CMakeFiles/VNlib.dir/event.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/event.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/event.cpp > CMakeFiles/VNlib.dir/event.i

VectorNav/src/CMakeFiles/VNlib.dir/event.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/event.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/event.cpp -o CMakeFiles/VNlib.dir/event.s

VectorNav/src/CMakeFiles/VNlib.dir/ezasyncdata.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/ezasyncdata.o: ../VectorNav/src/ezasyncdata.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/ezasyncdata.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/ezasyncdata.o -c /home/pi/CMakeListsTest/VectorNav/src/ezasyncdata.cpp

VectorNav/src/CMakeFiles/VNlib.dir/ezasyncdata.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/ezasyncdata.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/ezasyncdata.cpp > CMakeFiles/VNlib.dir/ezasyncdata.i

VectorNav/src/CMakeFiles/VNlib.dir/ezasyncdata.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/ezasyncdata.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/ezasyncdata.cpp -o CMakeFiles/VNlib.dir/ezasyncdata.s

VectorNav/src/CMakeFiles/VNlib.dir/memoryport.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/memoryport.o: ../VectorNav/src/memoryport.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/memoryport.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/memoryport.o -c /home/pi/CMakeListsTest/VectorNav/src/memoryport.cpp

VectorNav/src/CMakeFiles/VNlib.dir/memoryport.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/memoryport.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/memoryport.cpp > CMakeFiles/VNlib.dir/memoryport.i

VectorNav/src/CMakeFiles/VNlib.dir/memoryport.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/memoryport.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/memoryport.cpp -o CMakeFiles/VNlib.dir/memoryport.s

VectorNav/src/CMakeFiles/VNlib.dir/packet.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/packet.o: ../VectorNav/src/packet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/packet.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/packet.o -c /home/pi/CMakeListsTest/VectorNav/src/packet.cpp

VectorNav/src/CMakeFiles/VNlib.dir/packet.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/packet.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/packet.cpp > CMakeFiles/VNlib.dir/packet.i

VectorNav/src/CMakeFiles/VNlib.dir/packet.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/packet.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/packet.cpp -o CMakeFiles/VNlib.dir/packet.s

VectorNav/src/CMakeFiles/VNlib.dir/packetfinder.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/packetfinder.o: ../VectorNav/src/packetfinder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/packetfinder.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/packetfinder.o -c /home/pi/CMakeListsTest/VectorNav/src/packetfinder.cpp

VectorNav/src/CMakeFiles/VNlib.dir/packetfinder.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/packetfinder.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/packetfinder.cpp > CMakeFiles/VNlib.dir/packetfinder.i

VectorNav/src/CMakeFiles/VNlib.dir/packetfinder.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/packetfinder.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/packetfinder.cpp -o CMakeFiles/VNlib.dir/packetfinder.s

VectorNav/src/CMakeFiles/VNlib.dir/port.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/port.o: ../VectorNav/src/port.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/port.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/port.o -c /home/pi/CMakeListsTest/VectorNav/src/port.cpp

VectorNav/src/CMakeFiles/VNlib.dir/port.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/port.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/port.cpp > CMakeFiles/VNlib.dir/port.i

VectorNav/src/CMakeFiles/VNlib.dir/port.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/port.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/port.cpp -o CMakeFiles/VNlib.dir/port.s

VectorNav/src/CMakeFiles/VNlib.dir/position.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/position.o: ../VectorNav/src/position.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/position.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/position.o -c /home/pi/CMakeListsTest/VectorNav/src/position.cpp

VectorNav/src/CMakeFiles/VNlib.dir/position.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/position.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/position.cpp > CMakeFiles/VNlib.dir/position.i

VectorNav/src/CMakeFiles/VNlib.dir/position.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/position.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/position.cpp -o CMakeFiles/VNlib.dir/position.s

VectorNav/src/CMakeFiles/VNlib.dir/searcher.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/searcher.o: ../VectorNav/src/searcher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/searcher.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/searcher.o -c /home/pi/CMakeListsTest/VectorNav/src/searcher.cpp

VectorNav/src/CMakeFiles/VNlib.dir/searcher.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/searcher.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/searcher.cpp > CMakeFiles/VNlib.dir/searcher.i

VectorNav/src/CMakeFiles/VNlib.dir/searcher.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/searcher.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/searcher.cpp -o CMakeFiles/VNlib.dir/searcher.s

VectorNav/src/CMakeFiles/VNlib.dir/sensors.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/sensors.o: ../VectorNav/src/sensors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/sensors.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/sensors.o -c /home/pi/CMakeListsTest/VectorNav/src/sensors.cpp

VectorNav/src/CMakeFiles/VNlib.dir/sensors.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/sensors.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/sensors.cpp > CMakeFiles/VNlib.dir/sensors.i

VectorNav/src/CMakeFiles/VNlib.dir/sensors.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/sensors.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/sensors.cpp -o CMakeFiles/VNlib.dir/sensors.s

VectorNav/src/CMakeFiles/VNlib.dir/serialport.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/serialport.o: ../VectorNav/src/serialport.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/serialport.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/serialport.o -c /home/pi/CMakeListsTest/VectorNav/src/serialport.cpp

VectorNav/src/CMakeFiles/VNlib.dir/serialport.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/serialport.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/serialport.cpp > CMakeFiles/VNlib.dir/serialport.i

VectorNav/src/CMakeFiles/VNlib.dir/serialport.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/serialport.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/serialport.cpp -o CMakeFiles/VNlib.dir/serialport.s

VectorNav/src/CMakeFiles/VNlib.dir/thread.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/thread.o: ../VectorNav/src/thread.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/thread.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/thread.o -c /home/pi/CMakeListsTest/VectorNav/src/thread.cpp

VectorNav/src/CMakeFiles/VNlib.dir/thread.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/thread.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/thread.cpp > CMakeFiles/VNlib.dir/thread.i

VectorNav/src/CMakeFiles/VNlib.dir/thread.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/thread.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/thread.cpp -o CMakeFiles/VNlib.dir/thread.s

VectorNav/src/CMakeFiles/VNlib.dir/types.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/types.o: ../VectorNav/src/types.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/types.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/types.o -c /home/pi/CMakeListsTest/VectorNav/src/types.cpp

VectorNav/src/CMakeFiles/VNlib.dir/types.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/types.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/types.cpp > CMakeFiles/VNlib.dir/types.i

VectorNav/src/CMakeFiles/VNlib.dir/types.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/types.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/types.cpp -o CMakeFiles/VNlib.dir/types.s

VectorNav/src/CMakeFiles/VNlib.dir/util.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/util.o: ../VectorNav/src/util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/util.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/util.o -c /home/pi/CMakeListsTest/VectorNav/src/util.cpp

VectorNav/src/CMakeFiles/VNlib.dir/util.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/util.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/util.cpp > CMakeFiles/VNlib.dir/util.i

VectorNav/src/CMakeFiles/VNlib.dir/util.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/util.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/util.cpp -o CMakeFiles/VNlib.dir/util.s

VectorNav/src/CMakeFiles/VNlib.dir/utilities.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/utilities.o: ../VectorNav/src/utilities.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/utilities.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/utilities.o -c /home/pi/CMakeListsTest/VectorNav/src/utilities.cpp

VectorNav/src/CMakeFiles/VNlib.dir/utilities.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/utilities.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/utilities.cpp > CMakeFiles/VNlib.dir/utilities.i

VectorNav/src/CMakeFiles/VNlib.dir/utilities.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/utilities.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/utilities.cpp -o CMakeFiles/VNlib.dir/utilities.s

VectorNav/src/CMakeFiles/VNlib.dir/vntime.o: VectorNav/src/CMakeFiles/VNlib.dir/flags.make
VectorNav/src/CMakeFiles/VNlib.dir/vntime.o: ../VectorNav/src/vntime.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Building CXX object VectorNav/src/CMakeFiles/VNlib.dir/vntime.o"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VNlib.dir/vntime.o -c /home/pi/CMakeListsTest/VectorNav/src/vntime.cpp

VectorNav/src/CMakeFiles/VNlib.dir/vntime.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VNlib.dir/vntime.i"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMakeListsTest/VectorNav/src/vntime.cpp > CMakeFiles/VNlib.dir/vntime.i

VectorNav/src/CMakeFiles/VNlib.dir/vntime.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VNlib.dir/vntime.s"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMakeListsTest/VectorNav/src/vntime.cpp -o CMakeFiles/VNlib.dir/vntime.s

# Object files for target VNlib
VNlib_OBJECTS = \
"CMakeFiles/VNlib.dir/attitude.o" \
"CMakeFiles/VNlib.dir/compositedata.o" \
"CMakeFiles/VNlib.dir/conversions.o" \
"CMakeFiles/VNlib.dir/criticalsection.o" \
"CMakeFiles/VNlib.dir/dllvalidator.o" \
"CMakeFiles/VNlib.dir/event.o" \
"CMakeFiles/VNlib.dir/ezasyncdata.o" \
"CMakeFiles/VNlib.dir/memoryport.o" \
"CMakeFiles/VNlib.dir/packet.o" \
"CMakeFiles/VNlib.dir/packetfinder.o" \
"CMakeFiles/VNlib.dir/port.o" \
"CMakeFiles/VNlib.dir/position.o" \
"CMakeFiles/VNlib.dir/searcher.o" \
"CMakeFiles/VNlib.dir/sensors.o" \
"CMakeFiles/VNlib.dir/serialport.o" \
"CMakeFiles/VNlib.dir/thread.o" \
"CMakeFiles/VNlib.dir/types.o" \
"CMakeFiles/VNlib.dir/util.o" \
"CMakeFiles/VNlib.dir/utilities.o" \
"CMakeFiles/VNlib.dir/vntime.o"

# External object files for target VNlib
VNlib_EXTERNAL_OBJECTS =

VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/attitude.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/compositedata.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/conversions.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/criticalsection.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/dllvalidator.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/event.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/ezasyncdata.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/memoryport.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/packet.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/packetfinder.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/port.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/position.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/searcher.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/sensors.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/serialport.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/thread.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/types.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/util.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/utilities.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/vntime.o
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/build.make
VectorNav/src/libVNlib.a: VectorNav/src/CMakeFiles/VNlib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/CMakeListsTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_21) "Linking CXX static library libVNlib.a"
	cd /home/pi/CMakeListsTest/build/VectorNav/src && $(CMAKE_COMMAND) -P CMakeFiles/VNlib.dir/cmake_clean_target.cmake
	cd /home/pi/CMakeListsTest/build/VectorNav/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/VNlib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
VectorNav/src/CMakeFiles/VNlib.dir/build: VectorNav/src/libVNlib.a

.PHONY : VectorNav/src/CMakeFiles/VNlib.dir/build

VectorNav/src/CMakeFiles/VNlib.dir/clean:
	cd /home/pi/CMakeListsTest/build/VectorNav/src && $(CMAKE_COMMAND) -P CMakeFiles/VNlib.dir/cmake_clean.cmake
.PHONY : VectorNav/src/CMakeFiles/VNlib.dir/clean

VectorNav/src/CMakeFiles/VNlib.dir/depend:
	cd /home/pi/CMakeListsTest/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/CMakeListsTest /home/pi/CMakeListsTest/VectorNav/src /home/pi/CMakeListsTest/build /home/pi/CMakeListsTest/build/VectorNav/src /home/pi/CMakeListsTest/build/VectorNav/src/CMakeFiles/VNlib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : VectorNav/src/CMakeFiles/VNlib.dir/depend

