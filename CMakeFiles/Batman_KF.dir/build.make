# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.3

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.3.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.3.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/SeanSmith/Junior/cs585/a4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/SeanSmith/Junior/cs585/a4

# Include any dependencies generated for this target.
include CMakeFiles/Batman_KF.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Batman_KF.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Batman_KF.dir/flags.make

CMakeFiles/Batman_KF.dir/Tracker.cpp.o: CMakeFiles/Batman_KF.dir/flags.make
CMakeFiles/Batman_KF.dir/Tracker.cpp.o: Tracker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/SeanSmith/Junior/cs585/a4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Batman_KF.dir/Tracker.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Batman_KF.dir/Tracker.cpp.o -c /Users/SeanSmith/Junior/cs585/a4/Tracker.cpp

CMakeFiles/Batman_KF.dir/Tracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Batman_KF.dir/Tracker.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/SeanSmith/Junior/cs585/a4/Tracker.cpp > CMakeFiles/Batman_KF.dir/Tracker.cpp.i

CMakeFiles/Batman_KF.dir/Tracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Batman_KF.dir/Tracker.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/SeanSmith/Junior/cs585/a4/Tracker.cpp -o CMakeFiles/Batman_KF.dir/Tracker.cpp.s

CMakeFiles/Batman_KF.dir/Tracker.cpp.o.requires:

.PHONY : CMakeFiles/Batman_KF.dir/Tracker.cpp.o.requires

CMakeFiles/Batman_KF.dir/Tracker.cpp.o.provides: CMakeFiles/Batman_KF.dir/Tracker.cpp.o.requires
	$(MAKE) -f CMakeFiles/Batman_KF.dir/build.make CMakeFiles/Batman_KF.dir/Tracker.cpp.o.provides.build
.PHONY : CMakeFiles/Batman_KF.dir/Tracker.cpp.o.provides

CMakeFiles/Batman_KF.dir/Tracker.cpp.o.provides.build: CMakeFiles/Batman_KF.dir/Tracker.cpp.o


CMakeFiles/Batman_KF.dir/Kalman.cpp.o: CMakeFiles/Batman_KF.dir/flags.make
CMakeFiles/Batman_KF.dir/Kalman.cpp.o: Kalman.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/SeanSmith/Junior/cs585/a4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Batman_KF.dir/Kalman.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Batman_KF.dir/Kalman.cpp.o -c /Users/SeanSmith/Junior/cs585/a4/Kalman.cpp

CMakeFiles/Batman_KF.dir/Kalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Batman_KF.dir/Kalman.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/SeanSmith/Junior/cs585/a4/Kalman.cpp > CMakeFiles/Batman_KF.dir/Kalman.cpp.i

CMakeFiles/Batman_KF.dir/Kalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Batman_KF.dir/Kalman.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/SeanSmith/Junior/cs585/a4/Kalman.cpp -o CMakeFiles/Batman_KF.dir/Kalman.cpp.s

CMakeFiles/Batman_KF.dir/Kalman.cpp.o.requires:

.PHONY : CMakeFiles/Batman_KF.dir/Kalman.cpp.o.requires

CMakeFiles/Batman_KF.dir/Kalman.cpp.o.provides: CMakeFiles/Batman_KF.dir/Kalman.cpp.o.requires
	$(MAKE) -f CMakeFiles/Batman_KF.dir/build.make CMakeFiles/Batman_KF.dir/Kalman.cpp.o.provides.build
.PHONY : CMakeFiles/Batman_KF.dir/Kalman.cpp.o.provides

CMakeFiles/Batman_KF.dir/Kalman.cpp.o.provides.build: CMakeFiles/Batman_KF.dir/Kalman.cpp.o


CMakeFiles/Batman_KF.dir/Hungarian.cpp.o: CMakeFiles/Batman_KF.dir/flags.make
CMakeFiles/Batman_KF.dir/Hungarian.cpp.o: Hungarian.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/SeanSmith/Junior/cs585/a4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Batman_KF.dir/Hungarian.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Batman_KF.dir/Hungarian.cpp.o -c /Users/SeanSmith/Junior/cs585/a4/Hungarian.cpp

CMakeFiles/Batman_KF.dir/Hungarian.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Batman_KF.dir/Hungarian.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/SeanSmith/Junior/cs585/a4/Hungarian.cpp > CMakeFiles/Batman_KF.dir/Hungarian.cpp.i

CMakeFiles/Batman_KF.dir/Hungarian.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Batman_KF.dir/Hungarian.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/SeanSmith/Junior/cs585/a4/Hungarian.cpp -o CMakeFiles/Batman_KF.dir/Hungarian.cpp.s

CMakeFiles/Batman_KF.dir/Hungarian.cpp.o.requires:

.PHONY : CMakeFiles/Batman_KF.dir/Hungarian.cpp.o.requires

CMakeFiles/Batman_KF.dir/Hungarian.cpp.o.provides: CMakeFiles/Batman_KF.dir/Hungarian.cpp.o.requires
	$(MAKE) -f CMakeFiles/Batman_KF.dir/build.make CMakeFiles/Batman_KF.dir/Hungarian.cpp.o.provides.build
.PHONY : CMakeFiles/Batman_KF.dir/Hungarian.cpp.o.provides

CMakeFiles/Batman_KF.dir/Hungarian.cpp.o.provides.build: CMakeFiles/Batman_KF.dir/Hungarian.cpp.o


CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o: CMakeFiles/Batman_KF.dir/flags.make
CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o: Batman_KF.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/SeanSmith/Junior/cs585/a4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o -c /Users/SeanSmith/Junior/cs585/a4/Batman_KF.cpp

CMakeFiles/Batman_KF.dir/Batman_KF.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Batman_KF.dir/Batman_KF.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/SeanSmith/Junior/cs585/a4/Batman_KF.cpp > CMakeFiles/Batman_KF.dir/Batman_KF.cpp.i

CMakeFiles/Batman_KF.dir/Batman_KF.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Batman_KF.dir/Batman_KF.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/SeanSmith/Junior/cs585/a4/Batman_KF.cpp -o CMakeFiles/Batman_KF.dir/Batman_KF.cpp.s

CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o.requires:

.PHONY : CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o.requires

CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o.provides: CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o.requires
	$(MAKE) -f CMakeFiles/Batman_KF.dir/build.make CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o.provides.build
.PHONY : CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o.provides

CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o.provides.build: CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o


# Object files for target Batman_KF
Batman_KF_OBJECTS = \
"CMakeFiles/Batman_KF.dir/Tracker.cpp.o" \
"CMakeFiles/Batman_KF.dir/Kalman.cpp.o" \
"CMakeFiles/Batman_KF.dir/Hungarian.cpp.o" \
"CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o"

# External object files for target Batman_KF
Batman_KF_EXTERNAL_OBJECTS =

Batman_KF: CMakeFiles/Batman_KF.dir/Tracker.cpp.o
Batman_KF: CMakeFiles/Batman_KF.dir/Kalman.cpp.o
Batman_KF: CMakeFiles/Batman_KF.dir/Hungarian.cpp.o
Batman_KF: CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o
Batman_KF: CMakeFiles/Batman_KF.dir/build.make
Batman_KF: /usr/local/lib/libopencv_videostab.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_ts.a
Batman_KF: /usr/local/lib/libopencv_superres.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_stitching.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_contrib.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_nonfree.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_ocl.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_gpu.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_photo.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_objdetect.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_legacy.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_video.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_ml.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_calib3d.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_features2d.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_highgui.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_imgproc.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_flann.2.4.12.dylib
Batman_KF: /usr/local/lib/libopencv_core.2.4.12.dylib
Batman_KF: CMakeFiles/Batman_KF.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/SeanSmith/Junior/cs585/a4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable Batman_KF"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Batman_KF.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Batman_KF.dir/build: Batman_KF

.PHONY : CMakeFiles/Batman_KF.dir/build

CMakeFiles/Batman_KF.dir/requires: CMakeFiles/Batman_KF.dir/Tracker.cpp.o.requires
CMakeFiles/Batman_KF.dir/requires: CMakeFiles/Batman_KF.dir/Kalman.cpp.o.requires
CMakeFiles/Batman_KF.dir/requires: CMakeFiles/Batman_KF.dir/Hungarian.cpp.o.requires
CMakeFiles/Batman_KF.dir/requires: CMakeFiles/Batman_KF.dir/Batman_KF.cpp.o.requires

.PHONY : CMakeFiles/Batman_KF.dir/requires

CMakeFiles/Batman_KF.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Batman_KF.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Batman_KF.dir/clean

CMakeFiles/Batman_KF.dir/depend:
	cd /Users/SeanSmith/Junior/cs585/a4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/SeanSmith/Junior/cs585/a4 /Users/SeanSmith/Junior/cs585/a4 /Users/SeanSmith/Junior/cs585/a4 /Users/SeanSmith/Junior/cs585/a4 /Users/SeanSmith/Junior/cs585/a4/CMakeFiles/Batman_KF.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Batman_KF.dir/depend

