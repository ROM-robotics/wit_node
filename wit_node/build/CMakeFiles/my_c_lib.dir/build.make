# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/mr_robot/test/ws/src/wit_node

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mr_robot/test/ws/src/wit_node/build

# Include any dependencies generated for this target.
include CMakeFiles/my_c_lib.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/my_c_lib.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/my_c_lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my_c_lib.dir/flags.make

CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.o: CMakeFiles/my_c_lib.dir/flags.make
CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.o: ../src/wit_c_sdk.c
CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.o: CMakeFiles/my_c_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mr_robot/test/ws/src/wit_node/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.o -MF CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.o.d -o CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.o -c /home/mr_robot/test/ws/src/wit_node/src/wit_c_sdk.c

CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/mr_robot/test/ws/src/wit_node/src/wit_c_sdk.c > CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.i

CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/mr_robot/test/ws/src/wit_node/src/wit_c_sdk.c -o CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.s

# Object files for target my_c_lib
my_c_lib_OBJECTS = \
"CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.o"

# External object files for target my_c_lib
my_c_lib_EXTERNAL_OBJECTS =

libmy_c_lib.a: CMakeFiles/my_c_lib.dir/src/wit_c_sdk.c.o
libmy_c_lib.a: CMakeFiles/my_c_lib.dir/build.make
libmy_c_lib.a: CMakeFiles/my_c_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mr_robot/test/ws/src/wit_node/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libmy_c_lib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/my_c_lib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_c_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my_c_lib.dir/build: libmy_c_lib.a
.PHONY : CMakeFiles/my_c_lib.dir/build

CMakeFiles/my_c_lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my_c_lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my_c_lib.dir/clean

CMakeFiles/my_c_lib.dir/depend:
	cd /home/mr_robot/test/ws/src/wit_node/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mr_robot/test/ws/src/wit_node /home/mr_robot/test/ws/src/wit_node /home/mr_robot/test/ws/src/wit_node/build /home/mr_robot/test/ws/src/wit_node/build /home/mr_robot/test/ws/src/wit_node/build/CMakeFiles/my_c_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/my_c_lib.dir/depend

