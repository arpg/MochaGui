# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

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
CMAKE_COMMAND = /home/arpg/temp/cmake-3.23.0/bin/cmake

# The command to remove a file.
RM = /home/arpg/temp/cmake-3.23.0/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/arpg/mocha_ws/src/mochagui

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/arpg/mocha_ws/src/mochagui/build

# Include any dependencies generated for this target.
include NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/compiler_depend.make

# Include the progress variables for this target.
include NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/progress.make

# Include the compile flags for this target's objects.
include NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/flags.make

NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvWidgets.cpp.o: NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/flags.make
NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvWidgets.cpp.o: ../NvWidgets/nvwidgets/nvWidgets.cpp
NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvWidgets.cpp.o: NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/arpg/mocha_ws/src/mochagui/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvWidgets.cpp.o"
	cd /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvWidgets.cpp.o -MF CMakeFiles/nvwidgets.dir/nvWidgets.cpp.o.d -o CMakeFiles/nvwidgets.dir/nvWidgets.cpp.o -c /home/arpg/mocha_ws/src/mochagui/NvWidgets/nvwidgets/nvWidgets.cpp

NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvWidgets.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nvwidgets.dir/nvWidgets.cpp.i"
	cd /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/arpg/mocha_ws/src/mochagui/NvWidgets/nvwidgets/nvWidgets.cpp > CMakeFiles/nvwidgets.dir/nvWidgets.cpp.i

NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvWidgets.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nvwidgets.dir/nvWidgets.cpp.s"
	cd /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/arpg/mocha_ws/src/mochagui/NvWidgets/nvwidgets/nvWidgets.cpp -o CMakeFiles/nvwidgets.dir/nvWidgets.cpp.s

NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.o: NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/flags.make
NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.o: ../NvWidgets/nvwidgets/nvGLWidgets.cpp
NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.o: NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/arpg/mocha_ws/src/mochagui/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.o"
	cd /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.o -MF CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.o.d -o CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.o -c /home/arpg/mocha_ws/src/mochagui/NvWidgets/nvwidgets/nvGLWidgets.cpp

NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.i"
	cd /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/arpg/mocha_ws/src/mochagui/NvWidgets/nvwidgets/nvGLWidgets.cpp > CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.i

NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.s"
	cd /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/arpg/mocha_ws/src/mochagui/NvWidgets/nvwidgets/nvGLWidgets.cpp -o CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.s

NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.o: NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/flags.make
NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.o: ../NvWidgets/nvwidgets/nvGlutWidgets.cpp
NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.o: NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/arpg/mocha_ws/src/mochagui/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.o"
	cd /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.o -MF CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.o.d -o CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.o -c /home/arpg/mocha_ws/src/mochagui/NvWidgets/nvwidgets/nvGlutWidgets.cpp

NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.i"
	cd /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/arpg/mocha_ws/src/mochagui/NvWidgets/nvwidgets/nvGlutWidgets.cpp > CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.i

NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.s"
	cd /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/arpg/mocha_ws/src/mochagui/NvWidgets/nvwidgets/nvGlutWidgets.cpp -o CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.s

# Object files for target nvwidgets
nvwidgets_OBJECTS = \
"CMakeFiles/nvwidgets.dir/nvWidgets.cpp.o" \
"CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.o" \
"CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.o"

# External object files for target nvwidgets
nvwidgets_EXTERNAL_OBJECTS =

devel/lib/libnvwidgets.so: NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvWidgets.cpp.o
devel/lib/libnvwidgets.so: NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGLWidgets.cpp.o
devel/lib/libnvwidgets.so: NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/nvGlutWidgets.cpp.o
devel/lib/libnvwidgets.so: NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/build.make
devel/lib/libnvwidgets.so: /usr/lib/aarch64-linux-gnu/libGLEW.so
devel/lib/libnvwidgets.so: /usr/lib/aarch64-linux-gnu/libglut.so
devel/lib/libnvwidgets.so: /usr/lib/aarch64-linux-gnu/libXmu.so
devel/lib/libnvwidgets.so: /usr/lib/aarch64-linux-gnu/libXi.so
devel/lib/libnvwidgets.so: /usr/lib/aarch64-linux-gnu/libGL.so
devel/lib/libnvwidgets.so: /usr/lib/aarch64-linux-gnu/libGLU.so
devel/lib/libnvwidgets.so: NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/arpg/mocha_ws/src/mochagui/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library ../../devel/lib/libnvwidgets.so"
	cd /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/nvwidgets.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/build: devel/lib/libnvwidgets.so
.PHONY : NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/build

NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/clean:
	cd /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets && $(CMAKE_COMMAND) -P CMakeFiles/nvwidgets.dir/cmake_clean.cmake
.PHONY : NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/clean

NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/depend:
	cd /home/arpg/mocha_ws/src/mochagui/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/arpg/mocha_ws/src/mochagui /home/arpg/mocha_ws/src/mochagui/NvWidgets/nvwidgets /home/arpg/mocha_ws/src/mochagui/build /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets /home/arpg/mocha_ws/src/mochagui/build/NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : NvWidgets/nvwidgets/CMakeFiles/nvwidgets.dir/depend

