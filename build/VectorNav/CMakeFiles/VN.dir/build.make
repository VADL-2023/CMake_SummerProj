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
include VectorNav/CMakeFiles/VN.dir/depend.make

# Include the progress variables for this target.
include VectorNav/CMakeFiles/VN.dir/progress.make

# Include the compile flags for this target's objects.
include VectorNav/CMakeFiles/VN.dir/flags.make

VectorNav/CMakeFiles/VN.dir/src/attitude.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/attitude.o: ../VectorNav/src/attitude.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/attitude.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/attitude.o -c /home/pi/CMake_SummerProj/VectorNav/src/attitude.cpp

VectorNav/CMakeFiles/VN.dir/src/attitude.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/attitude.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/attitude.cpp > CMakeFiles/VN.dir/src/attitude.i

VectorNav/CMakeFiles/VN.dir/src/attitude.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/attitude.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/attitude.cpp -o CMakeFiles/VN.dir/src/attitude.s

VectorNav/CMakeFiles/VN.dir/src/compositedata.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/compositedata.o: ../VectorNav/src/compositedata.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/compositedata.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/compositedata.o -c /home/pi/CMake_SummerProj/VectorNav/src/compositedata.cpp

VectorNav/CMakeFiles/VN.dir/src/compositedata.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/compositedata.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/compositedata.cpp > CMakeFiles/VN.dir/src/compositedata.i

VectorNav/CMakeFiles/VN.dir/src/compositedata.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/compositedata.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/compositedata.cpp -o CMakeFiles/VN.dir/src/compositedata.s

VectorNav/CMakeFiles/VN.dir/src/conversions.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/conversions.o: ../VectorNav/src/conversions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/conversions.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/conversions.o -c /home/pi/CMake_SummerProj/VectorNav/src/conversions.cpp

VectorNav/CMakeFiles/VN.dir/src/conversions.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/conversions.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/conversions.cpp > CMakeFiles/VN.dir/src/conversions.i

VectorNav/CMakeFiles/VN.dir/src/conversions.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/conversions.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/conversions.cpp -o CMakeFiles/VN.dir/src/conversions.s

VectorNav/CMakeFiles/VN.dir/src/criticalsection.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/criticalsection.o: ../VectorNav/src/criticalsection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/criticalsection.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/criticalsection.o -c /home/pi/CMake_SummerProj/VectorNav/src/criticalsection.cpp

VectorNav/CMakeFiles/VN.dir/src/criticalsection.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/criticalsection.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/criticalsection.cpp > CMakeFiles/VN.dir/src/criticalsection.i

VectorNav/CMakeFiles/VN.dir/src/criticalsection.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/criticalsection.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/criticalsection.cpp -o CMakeFiles/VN.dir/src/criticalsection.s

VectorNav/CMakeFiles/VN.dir/src/dllvalidator.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/dllvalidator.o: ../VectorNav/src/dllvalidator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/dllvalidator.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/dllvalidator.o -c /home/pi/CMake_SummerProj/VectorNav/src/dllvalidator.cpp

VectorNav/CMakeFiles/VN.dir/src/dllvalidator.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/dllvalidator.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/dllvalidator.cpp > CMakeFiles/VN.dir/src/dllvalidator.i

VectorNav/CMakeFiles/VN.dir/src/dllvalidator.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/dllvalidator.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/dllvalidator.cpp -o CMakeFiles/VN.dir/src/dllvalidator.s

VectorNav/CMakeFiles/VN.dir/src/event.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/event.o: ../VectorNav/src/event.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/event.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/event.o -c /home/pi/CMake_SummerProj/VectorNav/src/event.cpp

VectorNav/CMakeFiles/VN.dir/src/event.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/event.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/event.cpp > CMakeFiles/VN.dir/src/event.i

VectorNav/CMakeFiles/VN.dir/src/event.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/event.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/event.cpp -o CMakeFiles/VN.dir/src/event.s

VectorNav/CMakeFiles/VN.dir/src/error_detection.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/error_detection.o: ../VectorNav/src/error_detection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/error_detection.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/error_detection.o -c /home/pi/CMake_SummerProj/VectorNav/src/error_detection.cpp

VectorNav/CMakeFiles/VN.dir/src/error_detection.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/error_detection.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/error_detection.cpp > CMakeFiles/VN.dir/src/error_detection.i

VectorNav/CMakeFiles/VN.dir/src/error_detection.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/error_detection.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/error_detection.cpp -o CMakeFiles/VN.dir/src/error_detection.s

VectorNav/CMakeFiles/VN.dir/src/ezasyncdata.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/ezasyncdata.o: ../VectorNav/src/ezasyncdata.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/ezasyncdata.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/ezasyncdata.o -c /home/pi/CMake_SummerProj/VectorNav/src/ezasyncdata.cpp

VectorNav/CMakeFiles/VN.dir/src/ezasyncdata.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/ezasyncdata.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/ezasyncdata.cpp > CMakeFiles/VN.dir/src/ezasyncdata.i

VectorNav/CMakeFiles/VN.dir/src/ezasyncdata.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/ezasyncdata.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/ezasyncdata.cpp -o CMakeFiles/VN.dir/src/ezasyncdata.s

VectorNav/CMakeFiles/VN.dir/src/memoryport.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/memoryport.o: ../VectorNav/src/memoryport.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/memoryport.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/memoryport.o -c /home/pi/CMake_SummerProj/VectorNav/src/memoryport.cpp

VectorNav/CMakeFiles/VN.dir/src/memoryport.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/memoryport.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/memoryport.cpp > CMakeFiles/VN.dir/src/memoryport.i

VectorNav/CMakeFiles/VN.dir/src/memoryport.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/memoryport.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/memoryport.cpp -o CMakeFiles/VN.dir/src/memoryport.s

VectorNav/CMakeFiles/VN.dir/src/packet.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/packet.o: ../VectorNav/src/packet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/packet.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/packet.o -c /home/pi/CMake_SummerProj/VectorNav/src/packet.cpp

VectorNav/CMakeFiles/VN.dir/src/packet.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/packet.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/packet.cpp > CMakeFiles/VN.dir/src/packet.i

VectorNav/CMakeFiles/VN.dir/src/packet.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/packet.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/packet.cpp -o CMakeFiles/VN.dir/src/packet.s

VectorNav/CMakeFiles/VN.dir/src/packetfinder.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/packetfinder.o: ../VectorNav/src/packetfinder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/packetfinder.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/packetfinder.o -c /home/pi/CMake_SummerProj/VectorNav/src/packetfinder.cpp

VectorNav/CMakeFiles/VN.dir/src/packetfinder.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/packetfinder.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/packetfinder.cpp > CMakeFiles/VN.dir/src/packetfinder.i

VectorNav/CMakeFiles/VN.dir/src/packetfinder.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/packetfinder.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/packetfinder.cpp -o CMakeFiles/VN.dir/src/packetfinder.s

VectorNav/CMakeFiles/VN.dir/src/port.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/port.o: ../VectorNav/src/port.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/port.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/port.o -c /home/pi/CMake_SummerProj/VectorNav/src/port.cpp

VectorNav/CMakeFiles/VN.dir/src/port.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/port.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/port.cpp > CMakeFiles/VN.dir/src/port.i

VectorNav/CMakeFiles/VN.dir/src/port.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/port.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/port.cpp -o CMakeFiles/VN.dir/src/port.s

VectorNav/CMakeFiles/VN.dir/src/position.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/position.o: ../VectorNav/src/position.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/position.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/position.o -c /home/pi/CMake_SummerProj/VectorNav/src/position.cpp

VectorNav/CMakeFiles/VN.dir/src/position.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/position.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/position.cpp > CMakeFiles/VN.dir/src/position.i

VectorNav/CMakeFiles/VN.dir/src/position.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/position.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/position.cpp -o CMakeFiles/VN.dir/src/position.s

VectorNav/CMakeFiles/VN.dir/src/searcher.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/searcher.o: ../VectorNav/src/searcher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/searcher.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/searcher.o -c /home/pi/CMake_SummerProj/VectorNav/src/searcher.cpp

VectorNav/CMakeFiles/VN.dir/src/searcher.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/searcher.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/searcher.cpp > CMakeFiles/VN.dir/src/searcher.i

VectorNav/CMakeFiles/VN.dir/src/searcher.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/searcher.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/searcher.cpp -o CMakeFiles/VN.dir/src/searcher.s

VectorNav/CMakeFiles/VN.dir/src/sensors.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/sensors.o: ../VectorNav/src/sensors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/sensors.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/sensors.o -c /home/pi/CMake_SummerProj/VectorNav/src/sensors.cpp

VectorNav/CMakeFiles/VN.dir/src/sensors.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/sensors.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/sensors.cpp > CMakeFiles/VN.dir/src/sensors.i

VectorNav/CMakeFiles/VN.dir/src/sensors.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/sensors.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/sensors.cpp -o CMakeFiles/VN.dir/src/sensors.s

VectorNav/CMakeFiles/VN.dir/src/serialport.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/serialport.o: ../VectorNav/src/serialport.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/serialport.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/serialport.o -c /home/pi/CMake_SummerProj/VectorNav/src/serialport.cpp

VectorNav/CMakeFiles/VN.dir/src/serialport.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/serialport.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/serialport.cpp > CMakeFiles/VN.dir/src/serialport.i

VectorNav/CMakeFiles/VN.dir/src/serialport.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/serialport.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/serialport.cpp -o CMakeFiles/VN.dir/src/serialport.s

VectorNav/CMakeFiles/VN.dir/src/thread.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/thread.o: ../VectorNav/src/thread.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/thread.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/thread.o -c /home/pi/CMake_SummerProj/VectorNav/src/thread.cpp

VectorNav/CMakeFiles/VN.dir/src/thread.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/thread.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/thread.cpp > CMakeFiles/VN.dir/src/thread.i

VectorNav/CMakeFiles/VN.dir/src/thread.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/thread.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/thread.cpp -o CMakeFiles/VN.dir/src/thread.s

VectorNav/CMakeFiles/VN.dir/src/types.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/types.o: ../VectorNav/src/types.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/types.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/types.o -c /home/pi/CMake_SummerProj/VectorNav/src/types.cpp

VectorNav/CMakeFiles/VN.dir/src/types.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/types.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/types.cpp > CMakeFiles/VN.dir/src/types.i

VectorNav/CMakeFiles/VN.dir/src/types.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/types.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/types.cpp -o CMakeFiles/VN.dir/src/types.s

VectorNav/CMakeFiles/VN.dir/src/util.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/util.o: ../VectorNav/src/util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/util.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/util.o -c /home/pi/CMake_SummerProj/VectorNav/src/util.cpp

VectorNav/CMakeFiles/VN.dir/src/util.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/util.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/util.cpp > CMakeFiles/VN.dir/src/util.i

VectorNav/CMakeFiles/VN.dir/src/util.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/util.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/util.cpp -o CMakeFiles/VN.dir/src/util.s

VectorNav/CMakeFiles/VN.dir/src/utilities.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/utilities.o: ../VectorNav/src/utilities.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/utilities.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/utilities.o -c /home/pi/CMake_SummerProj/VectorNav/src/utilities.cpp

VectorNav/CMakeFiles/VN.dir/src/utilities.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/utilities.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/utilities.cpp > CMakeFiles/VN.dir/src/utilities.i

VectorNav/CMakeFiles/VN.dir/src/utilities.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/utilities.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/utilities.cpp -o CMakeFiles/VN.dir/src/utilities.s

VectorNav/CMakeFiles/VN.dir/src/vntime.o: VectorNav/CMakeFiles/VN.dir/flags.make
VectorNav/CMakeFiles/VN.dir/src/vntime.o: ../VectorNav/src/vntime.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_21) "Building CXX object VectorNav/CMakeFiles/VN.dir/src/vntime.o"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VN.dir/src/vntime.o -c /home/pi/CMake_SummerProj/VectorNav/src/vntime.cpp

VectorNav/CMakeFiles/VN.dir/src/vntime.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VN.dir/src/vntime.i"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/CMake_SummerProj/VectorNav/src/vntime.cpp > CMakeFiles/VN.dir/src/vntime.i

VectorNav/CMakeFiles/VN.dir/src/vntime.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VN.dir/src/vntime.s"
	cd /home/pi/CMake_SummerProj/build/VectorNav && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/CMake_SummerProj/VectorNav/src/vntime.cpp -o CMakeFiles/VN.dir/src/vntime.s

# Object files for target VN
VN_OBJECTS = \
"CMakeFiles/VN.dir/src/attitude.o" \
"CMakeFiles/VN.dir/src/compositedata.o" \
"CMakeFiles/VN.dir/src/conversions.o" \
"CMakeFiles/VN.dir/src/criticalsection.o" \
"CMakeFiles/VN.dir/src/dllvalidator.o" \
"CMakeFiles/VN.dir/src/event.o" \
"CMakeFiles/VN.dir/src/error_detection.o" \
"CMakeFiles/VN.dir/src/ezasyncdata.o" \
"CMakeFiles/VN.dir/src/memoryport.o" \
"CMakeFiles/VN.dir/src/packet.o" \
"CMakeFiles/VN.dir/src/packetfinder.o" \
"CMakeFiles/VN.dir/src/port.o" \
"CMakeFiles/VN.dir/src/position.o" \
"CMakeFiles/VN.dir/src/searcher.o" \
"CMakeFiles/VN.dir/src/sensors.o" \
"CMakeFiles/VN.dir/src/serialport.o" \
"CMakeFiles/VN.dir/src/thread.o" \
"CMakeFiles/VN.dir/src/types.o" \
"CMakeFiles/VN.dir/src/util.o" \
"CMakeFiles/VN.dir/src/utilities.o" \
"CMakeFiles/VN.dir/src/vntime.o"

# External object files for target VN
VN_EXTERNAL_OBJECTS =

VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/attitude.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/compositedata.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/conversions.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/criticalsection.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/dllvalidator.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/event.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/error_detection.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/ezasyncdata.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/memoryport.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/packet.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/packetfinder.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/port.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/position.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/searcher.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/sensors.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/serialport.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/thread.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/types.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/util.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/utilities.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/src/vntime.o
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/build.make
VectorNav/libVN.so: VectorNav/CMakeFiles/VN.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/CMake_SummerProj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_22) "Linking CXX shared library libVN.so"
	cd /home/pi/CMake_SummerProj/build/VectorNav && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/VN.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
VectorNav/CMakeFiles/VN.dir/build: VectorNav/libVN.so

.PHONY : VectorNav/CMakeFiles/VN.dir/build

VectorNav/CMakeFiles/VN.dir/clean:
	cd /home/pi/CMake_SummerProj/build/VectorNav && $(CMAKE_COMMAND) -P CMakeFiles/VN.dir/cmake_clean.cmake
.PHONY : VectorNav/CMakeFiles/VN.dir/clean

VectorNav/CMakeFiles/VN.dir/depend:
	cd /home/pi/CMake_SummerProj/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/CMake_SummerProj /home/pi/CMake_SummerProj/VectorNav /home/pi/CMake_SummerProj/build /home/pi/CMake_SummerProj/build/VectorNav /home/pi/CMake_SummerProj/build/VectorNav/CMakeFiles/VN.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : VectorNav/CMakeFiles/VN.dir/depend

