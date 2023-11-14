# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/khorshidi/git/pose_graph_optimization/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/khorshidi/git/pose_graph_optimization/src/build

# Include any dependencies generated for this target.
include utils/CMakeFiles/UtilLibrary.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include utils/CMakeFiles/UtilLibrary.dir/compiler_depend.make

# Include the progress variables for this target.
include utils/CMakeFiles/UtilLibrary.dir/progress.make

# Include the compile flags for this target's objects.
include utils/CMakeFiles/UtilLibrary.dir/flags.make

utils/CMakeFiles/UtilLibrary.dir/Pose2D.cpp.o: utils/CMakeFiles/UtilLibrary.dir/flags.make
utils/CMakeFiles/UtilLibrary.dir/Pose2D.cpp.o: /home/khorshidi/git/pose_graph_optimization/src/utils/Pose2D.cpp
utils/CMakeFiles/UtilLibrary.dir/Pose2D.cpp.o: utils/CMakeFiles/UtilLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khorshidi/git/pose_graph_optimization/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object utils/CMakeFiles/UtilLibrary.dir/Pose2D.cpp.o"
	cd /home/khorshidi/git/pose_graph_optimization/src/build/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utils/CMakeFiles/UtilLibrary.dir/Pose2D.cpp.o -MF CMakeFiles/UtilLibrary.dir/Pose2D.cpp.o.d -o CMakeFiles/UtilLibrary.dir/Pose2D.cpp.o -c /home/khorshidi/git/pose_graph_optimization/src/utils/Pose2D.cpp

utils/CMakeFiles/UtilLibrary.dir/Pose2D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UtilLibrary.dir/Pose2D.cpp.i"
	cd /home/khorshidi/git/pose_graph_optimization/src/build/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/khorshidi/git/pose_graph_optimization/src/utils/Pose2D.cpp > CMakeFiles/UtilLibrary.dir/Pose2D.cpp.i

utils/CMakeFiles/UtilLibrary.dir/Pose2D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UtilLibrary.dir/Pose2D.cpp.s"
	cd /home/khorshidi/git/pose_graph_optimization/src/build/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/khorshidi/git/pose_graph_optimization/src/utils/Pose2D.cpp -o CMakeFiles/UtilLibrary.dir/Pose2D.cpp.s

# Object files for target UtilLibrary
UtilLibrary_OBJECTS = \
"CMakeFiles/UtilLibrary.dir/Pose2D.cpp.o"

# External object files for target UtilLibrary
UtilLibrary_EXTERNAL_OBJECTS =

utils/libUtilLibrary.a: utils/CMakeFiles/UtilLibrary.dir/Pose2D.cpp.o
utils/libUtilLibrary.a: utils/CMakeFiles/UtilLibrary.dir/build.make
utils/libUtilLibrary.a: utils/CMakeFiles/UtilLibrary.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/khorshidi/git/pose_graph_optimization/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libUtilLibrary.a"
	cd /home/khorshidi/git/pose_graph_optimization/src/build/utils && $(CMAKE_COMMAND) -P CMakeFiles/UtilLibrary.dir/cmake_clean_target.cmake
	cd /home/khorshidi/git/pose_graph_optimization/src/build/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/UtilLibrary.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils/CMakeFiles/UtilLibrary.dir/build: utils/libUtilLibrary.a
.PHONY : utils/CMakeFiles/UtilLibrary.dir/build

utils/CMakeFiles/UtilLibrary.dir/clean:
	cd /home/khorshidi/git/pose_graph_optimization/src/build/utils && $(CMAKE_COMMAND) -P CMakeFiles/UtilLibrary.dir/cmake_clean.cmake
.PHONY : utils/CMakeFiles/UtilLibrary.dir/clean

utils/CMakeFiles/UtilLibrary.dir/depend:
	cd /home/khorshidi/git/pose_graph_optimization/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/khorshidi/git/pose_graph_optimization/src /home/khorshidi/git/pose_graph_optimization/src/utils /home/khorshidi/git/pose_graph_optimization/src/build /home/khorshidi/git/pose_graph_optimization/src/build/utils /home/khorshidi/git/pose_graph_optimization/src/build/utils/CMakeFiles/UtilLibrary.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils/CMakeFiles/UtilLibrary.dir/depend
