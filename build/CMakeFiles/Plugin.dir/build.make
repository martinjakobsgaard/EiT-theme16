# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/martin/workspace/rws-plugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/martin/workspace/rws-plugin/build

# Include any dependencies generated for this target.
include CMakeFiles/Plugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Plugin.dir/flags.make

CMakeFiles/Plugin.dir/Plugin.cpp.o: CMakeFiles/Plugin.dir/flags.make
CMakeFiles/Plugin.dir/Plugin.cpp.o: ../Plugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/martin/workspace/rws-plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Plugin.dir/Plugin.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Plugin.dir/Plugin.cpp.o -c /home/martin/workspace/rws-plugin/Plugin.cpp

CMakeFiles/Plugin.dir/Plugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Plugin.dir/Plugin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/martin/workspace/rws-plugin/Plugin.cpp > CMakeFiles/Plugin.dir/Plugin.cpp.i

CMakeFiles/Plugin.dir/Plugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Plugin.dir/Plugin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/martin/workspace/rws-plugin/Plugin.cpp -o CMakeFiles/Plugin.dir/Plugin.cpp.s

CMakeFiles/Plugin.dir/Plugin.cpp.o.requires:

.PHONY : CMakeFiles/Plugin.dir/Plugin.cpp.o.requires

CMakeFiles/Plugin.dir/Plugin.cpp.o.provides: CMakeFiles/Plugin.dir/Plugin.cpp.o.requires
	$(MAKE) -f CMakeFiles/Plugin.dir/build.make CMakeFiles/Plugin.dir/Plugin.cpp.o.provides.build
.PHONY : CMakeFiles/Plugin.dir/Plugin.cpp.o.provides

CMakeFiles/Plugin.dir/Plugin.cpp.o.provides.build: CMakeFiles/Plugin.dir/Plugin.cpp.o


CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o: CMakeFiles/Plugin.dir/flags.make
CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o: Plugin_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/martin/workspace/rws-plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o -c /home/martin/workspace/rws-plugin/build/Plugin_autogen/mocs_compilation.cpp

CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/martin/workspace/rws-plugin/build/Plugin_autogen/mocs_compilation.cpp > CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.i

CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/martin/workspace/rws-plugin/build/Plugin_autogen/mocs_compilation.cpp -o CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.s

CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o.requires:

.PHONY : CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o.requires

CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o.provides: CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o.requires
	$(MAKE) -f CMakeFiles/Plugin.dir/build.make CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o.provides.build
.PHONY : CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o.provides

CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o.provides.build: CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o


CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o: CMakeFiles/Plugin.dir/flags.make
CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o: Plugin_autogen/EWIEGA46WW/qrc_resources.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/martin/workspace/rws-plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o -c /home/martin/workspace/rws-plugin/build/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp

CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/martin/workspace/rws-plugin/build/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp > CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.i

CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/martin/workspace/rws-plugin/build/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp -o CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.s

CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o.requires:

.PHONY : CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o.requires

CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o.provides: CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o.requires
	$(MAKE) -f CMakeFiles/Plugin.dir/build.make CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o.provides.build
.PHONY : CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o.provides

CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o.provides.build: CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o


# Object files for target Plugin
Plugin_OBJECTS = \
"CMakeFiles/Plugin.dir/Plugin.cpp.o" \
"CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o"

# External object files for target Plugin
Plugin_EXTERNAL_OBJECTS =

../libs/libPlugin.so: CMakeFiles/Plugin.dir/Plugin.cpp.o
../libs/libPlugin.so: CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o
../libs/libPlugin.so: CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o
../libs/libPlugin.so: CMakeFiles/Plugin.dir/build.make
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurws_robworkstudioapp.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/rwsplugins/libsdurws_jog.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/rwsplugins/libsdurws_log.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/rwsplugins/libsdurws_playback.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/rwsplugins/libsdurws_propertyview.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/rwsplugins/libsdurws_treeview.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/rwsplugins/libsdurws_planning.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/rwsplugins/libsdurws_sensors.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/rwsplugins/libsdurws_workcelleditorplugin.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurws_workcelleditor.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/rwsplugins/libsdurws_luapl.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurws_luaeditor.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurws.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libqtpropertybrowser.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libyaobi.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libpqp.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/static/libsdurw_qhull.a
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/static/libsdurw_csgjs.a
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_algorithms.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_pathplanners.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_pathoptimization.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_simulation.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_opengl.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_assembly.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_task.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_calibration.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_csg.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_control.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_proximitystrategies.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_core.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_common.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_math.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libfcl.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/liblua5.3.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libm.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libassimp.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libGL.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libdl.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.9.5
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libglut.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libpython3.6m.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libyaobi.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libpqp.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/static/libsdurw_qhull.a
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/RobWork/static/libsdurw_csgjs.a
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_algorithms.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_pathplanners.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_pathoptimization.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_simulation.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_opengl.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_assembly.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_task.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_calibration.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_csg.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_control.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_proximitystrategies.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_core.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_common.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libsdurw_math.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libfcl.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/liblua5.3.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libm.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libassimp.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libGL.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libdl.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libglut.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libpython3.6m.so
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
../libs/libPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
../libs/libPlugin.so: CMakeFiles/Plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/martin/workspace/rws-plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared module ../libs/libPlugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Plugin.dir/build: ../libs/libPlugin.so

.PHONY : CMakeFiles/Plugin.dir/build

CMakeFiles/Plugin.dir/requires: CMakeFiles/Plugin.dir/Plugin.cpp.o.requires
CMakeFiles/Plugin.dir/requires: CMakeFiles/Plugin.dir/Plugin_autogen/mocs_compilation.cpp.o.requires
CMakeFiles/Plugin.dir/requires: CMakeFiles/Plugin.dir/Plugin_autogen/EWIEGA46WW/qrc_resources.cpp.o.requires

.PHONY : CMakeFiles/Plugin.dir/requires

CMakeFiles/Plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Plugin.dir/clean

CMakeFiles/Plugin.dir/depend:
	cd /home/martin/workspace/rws-plugin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/martin/workspace/rws-plugin /home/martin/workspace/rws-plugin /home/martin/workspace/rws-plugin/build /home/martin/workspace/rws-plugin/build /home/martin/workspace/rws-plugin/build/CMakeFiles/Plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Plugin.dir/depend

