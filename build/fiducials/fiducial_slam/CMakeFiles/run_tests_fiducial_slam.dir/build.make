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
CMAKE_SOURCE_DIR = /my_ros_data/pa_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /my_ros_data/pa_ws/build

# Utility rule file for run_tests_fiducial_slam.

# Include the progress variables for this target.
include fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam.dir/progress.make

run_tests_fiducial_slam: fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam.dir/build.make

.PHONY : run_tests_fiducial_slam

# Rule to build all files generated by this target.
fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam.dir/build: run_tests_fiducial_slam

.PHONY : fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam.dir/build

fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam.dir/clean:
	cd /my_ros_data/pa_ws/build/fiducials/fiducial_slam && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_fiducial_slam.dir/cmake_clean.cmake
.PHONY : fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam.dir/clean

fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam.dir/depend:
	cd /my_ros_data/pa_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /my_ros_data/pa_ws/src /my_ros_data/pa_ws/src/fiducials/fiducial_slam /my_ros_data/pa_ws/build /my_ros_data/pa_ws/build/fiducials/fiducial_slam /my_ros_data/pa_ws/build/fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : fiducials/fiducial_slam/CMakeFiles/run_tests_fiducial_slam.dir/depend

