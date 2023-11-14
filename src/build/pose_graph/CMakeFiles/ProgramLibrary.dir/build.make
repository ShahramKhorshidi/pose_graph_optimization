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
include pose_graph/CMakeFiles/ProgramLibrary.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include pose_graph/CMakeFiles/ProgramLibrary.dir/compiler_depend.make

# Include the progress variables for this target.
include pose_graph/CMakeFiles/ProgramLibrary.dir/progress.make

# Include the compile flags for this target's objects.
include pose_graph/CMakeFiles/ProgramLibrary.dir/flags.make

pose_graph/CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.o: pose_graph/CMakeFiles/ProgramLibrary.dir/flags.make
pose_graph/CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.o: /home/khorshidi/git/pose_graph_optimization/src/pose_graph/GraphConstraint.cpp
pose_graph/CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.o: pose_graph/CMakeFiles/ProgramLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khorshidi/git/pose_graph_optimization/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object pose_graph/CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.o"
	cd /home/khorshidi/git/pose_graph_optimization/src/build/pose_graph && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT pose_graph/CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.o -MF CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.o.d -o CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.o -c /home/khorshidi/git/pose_graph_optimization/src/pose_graph/GraphConstraint.cpp

pose_graph/CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.i"
	cd /home/khorshidi/git/pose_graph_optimization/src/build/pose_graph && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/khorshidi/git/pose_graph_optimization/src/pose_graph/GraphConstraint.cpp > CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.i

pose_graph/CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.s"
	cd /home/khorshidi/git/pose_graph_optimization/src/build/pose_graph && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/khorshidi/git/pose_graph_optimization/src/pose_graph/GraphConstraint.cpp -o CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.s

pose_graph/CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.o: pose_graph/CMakeFiles/ProgramLibrary.dir/flags.make
pose_graph/CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.o: /home/khorshidi/git/pose_graph_optimization/src/pose_graph/PoseGraph.cpp
pose_graph/CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.o: pose_graph/CMakeFiles/ProgramLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khorshidi/git/pose_graph_optimization/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object pose_graph/CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.o"
	cd /home/khorshidi/git/pose_graph_optimization/src/build/pose_graph && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT pose_graph/CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.o -MF CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.o.d -o CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.o -c /home/khorshidi/git/pose_graph_optimization/src/pose_graph/PoseGraph.cpp

pose_graph/CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.i"
	cd /home/khorshidi/git/pose_graph_optimization/src/build/pose_graph && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/khorshidi/git/pose_graph_optimization/src/pose_graph/PoseGraph.cpp > CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.i

pose_graph/CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.s"
	cd /home/khorshidi/git/pose_graph_optimization/src/build/pose_graph && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/khorshidi/git/pose_graph_optimization/src/pose_graph/PoseGraph.cpp -o CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.s

# Object files for target ProgramLibrary
ProgramLibrary_OBJECTS = \
"CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.o" \
"CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.o"

# External object files for target ProgramLibrary
ProgramLibrary_EXTERNAL_OBJECTS =

pose_graph/libProgramLibrary.a: pose_graph/CMakeFiles/ProgramLibrary.dir/GraphConstraint.cpp.o
pose_graph/libProgramLibrary.a: pose_graph/CMakeFiles/ProgramLibrary.dir/PoseGraph.cpp.o
pose_graph/libProgramLibrary.a: pose_graph/CMakeFiles/ProgramLibrary.dir/build.make
pose_graph/libProgramLibrary.a: pose_graph/CMakeFiles/ProgramLibrary.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/khorshidi/git/pose_graph_optimization/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libProgramLibrary.a"
	cd /home/khorshidi/git/pose_graph_optimization/src/build/pose_graph && $(CMAKE_COMMAND) -P CMakeFiles/ProgramLibrary.dir/cmake_clean_target.cmake
	cd /home/khorshidi/git/pose_graph_optimization/src/build/pose_graph && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ProgramLibrary.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pose_graph/CMakeFiles/ProgramLibrary.dir/build: pose_graph/libProgramLibrary.a
.PHONY : pose_graph/CMakeFiles/ProgramLibrary.dir/build

pose_graph/CMakeFiles/ProgramLibrary.dir/clean:
	cd /home/khorshidi/git/pose_graph_optimization/src/build/pose_graph && $(CMAKE_COMMAND) -P CMakeFiles/ProgramLibrary.dir/cmake_clean.cmake
.PHONY : pose_graph/CMakeFiles/ProgramLibrary.dir/clean

pose_graph/CMakeFiles/ProgramLibrary.dir/depend:
	cd /home/khorshidi/git/pose_graph_optimization/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/khorshidi/git/pose_graph_optimization/src /home/khorshidi/git/pose_graph_optimization/src/pose_graph /home/khorshidi/git/pose_graph_optimization/src/build /home/khorshidi/git/pose_graph_optimization/src/build/pose_graph /home/khorshidi/git/pose_graph_optimization/src/build/pose_graph/CMakeFiles/ProgramLibrary.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pose_graph/CMakeFiles/ProgramLibrary.dir/depend

