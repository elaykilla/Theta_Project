# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/elay/workspace/OpencvTest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/elay/workspace/OpencvTest

# Include any dependencies generated for this target.
include CMakeFiles/load.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/load.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/load.dir/flags.make

CMakeFiles/load.dir/load.cpp.o: CMakeFiles/load.dir/flags.make
CMakeFiles/load.dir/load.cpp.o: load.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/elay/workspace/OpencvTest/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/load.dir/load.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/load.dir/load.cpp.o -c /home/elay/workspace/OpencvTest/load.cpp

CMakeFiles/load.dir/load.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/load.dir/load.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/elay/workspace/OpencvTest/load.cpp > CMakeFiles/load.dir/load.cpp.i

CMakeFiles/load.dir/load.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/load.dir/load.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/elay/workspace/OpencvTest/load.cpp -o CMakeFiles/load.dir/load.cpp.s

CMakeFiles/load.dir/load.cpp.o.requires:
.PHONY : CMakeFiles/load.dir/load.cpp.o.requires

CMakeFiles/load.dir/load.cpp.o.provides: CMakeFiles/load.dir/load.cpp.o.requires
	$(MAKE) -f CMakeFiles/load.dir/build.make CMakeFiles/load.dir/load.cpp.o.provides.build
.PHONY : CMakeFiles/load.dir/load.cpp.o.provides

CMakeFiles/load.dir/load.cpp.o.provides.build: CMakeFiles/load.dir/load.cpp.o

# Object files for target load
load_OBJECTS = \
"CMakeFiles/load.dir/load.cpp.o"

# External object files for target load
load_EXTERNAL_OBJECTS =

load: CMakeFiles/load.dir/load.cpp.o
load: CMakeFiles/load.dir/build.make
load: /usr/lib/x86_64-linux-gnu/libboost_system.so
load: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
load: /usr/lib/x86_64-linux-gnu/libboost_thread.so
load: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
load: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
load: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
load: /usr/lib/x86_64-linux-gnu/libpthread.so
load: /usr/lib/libpcl_common.so
load: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
load: /usr/lib/libpcl_kdtree.so
load: /usr/lib/libpcl_octree.so
load: /usr/lib/libpcl_search.so
load: /usr/lib/x86_64-linux-gnu/libqhull.so
load: /usr/lib/libpcl_surface.so
load: /usr/lib/libpcl_sample_consensus.so
load: /usr/lib/libpcl_filters.so
load: /usr/lib/libpcl_features.so
load: /usr/lib/libpcl_segmentation.so
load: /usr/lib/libOpenNI.so
load: /usr/lib/libvtkCommon.so.5.8.0
load: /usr/lib/libvtkRendering.so.5.8.0
load: /usr/lib/libvtkHybrid.so.5.8.0
load: /usr/lib/libvtkCharts.so.5.8.0
load: /usr/lib/libpcl_io.so
load: /usr/lib/libpcl_registration.so
load: /usr/lib/libpcl_keypoints.so
load: /usr/lib/libpcl_recognition.so
load: /usr/lib/libpcl_visualization.so
load: /usr/lib/libpcl_people.so
load: /usr/lib/libpcl_outofcore.so
load: /usr/lib/libpcl_tracking.so
load: /usr/lib/libpcl_apps.so
load: /usr/lib/x86_64-linux-gnu/libboost_system.so
load: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
load: /usr/lib/x86_64-linux-gnu/libboost_thread.so
load: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
load: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
load: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
load: /usr/lib/x86_64-linux-gnu/libpthread.so
load: /usr/lib/x86_64-linux-gnu/libqhull.so
load: /usr/lib/libOpenNI.so
load: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
load: /usr/lib/libvtkCommon.so.5.8.0
load: /usr/lib/libvtkRendering.so.5.8.0
load: /usr/lib/libvtkHybrid.so.5.8.0
load: /usr/lib/libvtkCharts.so.5.8.0
load: /usr/local/lib/libopencv_videostab.so.2.4.8
load: /usr/local/lib/libopencv_video.so.2.4.8
load: /usr/local/lib/libopencv_ts.a
load: /usr/local/lib/libopencv_superres.so.2.4.8
load: /usr/local/lib/libopencv_stitching.so.2.4.8
load: /usr/local/lib/libopencv_photo.so.2.4.8
load: /usr/local/lib/libopencv_ocl.so.2.4.8
load: /usr/local/lib/libopencv_objdetect.so.2.4.8
load: /usr/local/lib/libopencv_nonfree.so.2.4.8
load: /usr/local/lib/libopencv_ml.so.2.4.8
load: /usr/local/lib/libopencv_legacy.so.2.4.8
load: /usr/local/lib/libopencv_imgproc.so.2.4.8
load: /usr/local/lib/libopencv_highgui.so.2.4.8
load: /usr/local/lib/libopencv_gpu.so.2.4.8
load: /usr/local/lib/libopencv_flann.so.2.4.8
load: /usr/local/lib/libopencv_features2d.so.2.4.8
load: /usr/local/lib/libopencv_core.so.2.4.8
load: /usr/local/lib/libopencv_contrib.so.2.4.8
load: /usr/local/lib/libopencv_calib3d.so.2.4.8
load: /usr/lib/x86_64-linux-gnu/libboost_log.a
load: /usr/lib/libpcl_common.so
load: /usr/lib/libpcl_kdtree.so
load: /usr/lib/libpcl_octree.so
load: /usr/lib/libpcl_search.so
load: /usr/lib/libpcl_surface.so
load: /usr/lib/libpcl_sample_consensus.so
load: /usr/lib/libpcl_filters.so
load: /usr/lib/libpcl_features.so
load: /usr/lib/libpcl_segmentation.so
load: /usr/lib/libpcl_io.so
load: /usr/lib/libpcl_registration.so
load: /usr/lib/libpcl_keypoints.so
load: /usr/lib/libpcl_recognition.so
load: /usr/lib/libpcl_visualization.so
load: /usr/lib/libpcl_people.so
load: /usr/lib/libpcl_outofcore.so
load: /usr/lib/libpcl_tracking.so
load: /usr/lib/libpcl_apps.so
load: /usr/lib/x86_64-linux-gnu/libboost_log.a
load: /usr/lib/libvtkViews.so.5.8.0
load: /usr/lib/libvtkInfovis.so.5.8.0
load: /usr/lib/libvtkWidgets.so.5.8.0
load: /usr/lib/libvtkHybrid.so.5.8.0
load: /usr/lib/libvtkParallel.so.5.8.0
load: /usr/lib/libvtkVolumeRendering.so.5.8.0
load: /usr/lib/libvtkRendering.so.5.8.0
load: /usr/lib/libvtkGraphics.so.5.8.0
load: /usr/lib/libvtkImaging.so.5.8.0
load: /usr/lib/libvtkIO.so.5.8.0
load: /usr/lib/libvtkFiltering.so.5.8.0
load: /usr/lib/libvtkCommon.so.5.8.0
load: /usr/lib/libvtksys.so.5.8.0
load: /usr/lib/x86_64-linux-gnu/libGLU.so
load: /usr/lib/x86_64-linux-gnu/libGL.so
load: /usr/lib/x86_64-linux-gnu/libSM.so
load: /usr/lib/x86_64-linux-gnu/libICE.so
load: /usr/lib/x86_64-linux-gnu/libX11.so
load: /usr/lib/x86_64-linux-gnu/libXext.so
load: /usr/local/lib/libopencv_nonfree.so.2.4.8
load: /usr/local/lib/libopencv_ocl.so.2.4.8
load: /usr/local/lib/libopencv_gpu.so.2.4.8
load: /usr/local/lib/libopencv_photo.so.2.4.8
load: /usr/local/lib/libopencv_objdetect.so.2.4.8
load: /usr/local/lib/libopencv_legacy.so.2.4.8
load: /usr/local/lib/libopencv_video.so.2.4.8
load: /usr/local/lib/libopencv_ml.so.2.4.8
load: /usr/local/lib/libopencv_calib3d.so.2.4.8
load: /usr/local/lib/libopencv_features2d.so.2.4.8
load: /usr/local/lib/libopencv_highgui.so.2.4.8
load: /usr/local/lib/libopencv_imgproc.so.2.4.8
load: /usr/local/lib/libopencv_flann.so.2.4.8
load: /usr/local/lib/libopencv_core.so.2.4.8
load: CMakeFiles/load.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable load"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/load.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/load.dir/build: load
.PHONY : CMakeFiles/load.dir/build

CMakeFiles/load.dir/requires: CMakeFiles/load.dir/load.cpp.o.requires
.PHONY : CMakeFiles/load.dir/requires

CMakeFiles/load.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/load.dir/cmake_clean.cmake
.PHONY : CMakeFiles/load.dir/clean

CMakeFiles/load.dir/depend:
	cd /home/elay/workspace/OpencvTest && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/elay/workspace/OpencvTest /home/elay/workspace/OpencvTest /home/elay/workspace/OpencvTest /home/elay/workspace/OpencvTest /home/elay/workspace/OpencvTest/CMakeFiles/load.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/load.dir/depend
