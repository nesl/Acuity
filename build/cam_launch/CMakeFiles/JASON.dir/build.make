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
CMAKE_SOURCE_DIR = /home/nesl/Desktop/Acuity/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nesl/Desktop/Acuity/build

# Include any dependencies generated for this target.
include cam_launch/CMakeFiles/JASON.dir/depend.make

# Include the progress variables for this target.
include cam_launch/CMakeFiles/JASON.dir/progress.make

# Include the compile flags for this target's objects.
include cam_launch/CMakeFiles/JASON.dir/flags.make

cam_launch/CMakeFiles/JASON.dir/src/KalmanTracker.cpp.o: cam_launch/CMakeFiles/JASON.dir/flags.make
cam_launch/CMakeFiles/JASON.dir/src/KalmanTracker.cpp.o: /home/nesl/Desktop/Acuity/src/cam_launch/src/KalmanTracker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nesl/Desktop/Acuity/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cam_launch/CMakeFiles/JASON.dir/src/KalmanTracker.cpp.o"
	cd /home/nesl/Desktop/Acuity/build/cam_launch && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/JASON.dir/src/KalmanTracker.cpp.o -c /home/nesl/Desktop/Acuity/src/cam_launch/src/KalmanTracker.cpp

cam_launch/CMakeFiles/JASON.dir/src/KalmanTracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/JASON.dir/src/KalmanTracker.cpp.i"
	cd /home/nesl/Desktop/Acuity/build/cam_launch && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nesl/Desktop/Acuity/src/cam_launch/src/KalmanTracker.cpp > CMakeFiles/JASON.dir/src/KalmanTracker.cpp.i

cam_launch/CMakeFiles/JASON.dir/src/KalmanTracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/JASON.dir/src/KalmanTracker.cpp.s"
	cd /home/nesl/Desktop/Acuity/build/cam_launch && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nesl/Desktop/Acuity/src/cam_launch/src/KalmanTracker.cpp -o CMakeFiles/JASON.dir/src/KalmanTracker.cpp.s

cam_launch/CMakeFiles/JASON.dir/src/Hungarian.cpp.o: cam_launch/CMakeFiles/JASON.dir/flags.make
cam_launch/CMakeFiles/JASON.dir/src/Hungarian.cpp.o: /home/nesl/Desktop/Acuity/src/cam_launch/src/Hungarian.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nesl/Desktop/Acuity/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object cam_launch/CMakeFiles/JASON.dir/src/Hungarian.cpp.o"
	cd /home/nesl/Desktop/Acuity/build/cam_launch && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/JASON.dir/src/Hungarian.cpp.o -c /home/nesl/Desktop/Acuity/src/cam_launch/src/Hungarian.cpp

cam_launch/CMakeFiles/JASON.dir/src/Hungarian.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/JASON.dir/src/Hungarian.cpp.i"
	cd /home/nesl/Desktop/Acuity/build/cam_launch && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nesl/Desktop/Acuity/src/cam_launch/src/Hungarian.cpp > CMakeFiles/JASON.dir/src/Hungarian.cpp.i

cam_launch/CMakeFiles/JASON.dir/src/Hungarian.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/JASON.dir/src/Hungarian.cpp.s"
	cd /home/nesl/Desktop/Acuity/build/cam_launch && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nesl/Desktop/Acuity/src/cam_launch/src/Hungarian.cpp -o CMakeFiles/JASON.dir/src/Hungarian.cpp.s

# Object files for target JASON
JASON_OBJECTS = \
"CMakeFiles/JASON.dir/src/KalmanTracker.cpp.o" \
"CMakeFiles/JASON.dir/src/Hungarian.cpp.o"

# External object files for target JASON
JASON_EXTERNAL_OBJECTS =

/home/nesl/Desktop/Acuity/devel/lib/libJASON.so: cam_launch/CMakeFiles/JASON.dir/src/KalmanTracker.cpp.o
/home/nesl/Desktop/Acuity/devel/lib/libJASON.so: cam_launch/CMakeFiles/JASON.dir/src/Hungarian.cpp.o
/home/nesl/Desktop/Acuity/devel/lib/libJASON.so: cam_launch/CMakeFiles/JASON.dir/build.make
/home/nesl/Desktop/Acuity/devel/lib/libJASON.so: cam_launch/CMakeFiles/JASON.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nesl/Desktop/Acuity/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library /home/nesl/Desktop/Acuity/devel/lib/libJASON.so"
	cd /home/nesl/Desktop/Acuity/build/cam_launch && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/JASON.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cam_launch/CMakeFiles/JASON.dir/build: /home/nesl/Desktop/Acuity/devel/lib/libJASON.so

.PHONY : cam_launch/CMakeFiles/JASON.dir/build

cam_launch/CMakeFiles/JASON.dir/clean:
	cd /home/nesl/Desktop/Acuity/build/cam_launch && $(CMAKE_COMMAND) -P CMakeFiles/JASON.dir/cmake_clean.cmake
.PHONY : cam_launch/CMakeFiles/JASON.dir/clean

cam_launch/CMakeFiles/JASON.dir/depend:
	cd /home/nesl/Desktop/Acuity/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nesl/Desktop/Acuity/src /home/nesl/Desktop/Acuity/src/cam_launch /home/nesl/Desktop/Acuity/build /home/nesl/Desktop/Acuity/build/cam_launch /home/nesl/Desktop/Acuity/build/cam_launch/CMakeFiles/JASON.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cam_launch/CMakeFiles/JASON.dir/depend

