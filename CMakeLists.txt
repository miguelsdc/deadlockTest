cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)


#===============================================================================
# INCLUDE ACTIONLIB
#===============================================================================
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

#===============================================================================
# ROS STUFF
#===============================================================================
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


#===============================================================================
# RPATH SETTINGS
#===============================================================================
# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH just yet
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


#===============================================================================
# COMPILER SETTINGS (Activate all Warnings)
#===============================================================================
# Taken from Stack Overflow (question: 2368811, user: mloskot)
# http://stackoverflow.com/q/2368811/935415
if(MSVC)
  # Force to always compile with W4
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    list(APPEND CMAKE_CXX_FLAGS "/W4")
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # Update if necessary
  list(APPEND CMAKE_CXX_FLAGS
    "-Wall -Wextra"
    )
endif()


#===============================================================================
# SOURCES, INCLUDES & LIBRARIES
#===============================================================================
#Find all source files (cpps and headers)
file(GLOB SOURCES "src/*.h" "src/*.hxx" "src/*.cpp" )

#Add include directories
include_directories( ${INCLUDES} )

#Add necessary definitions
add_definitions( ${DEFINES} )

#===============================================================================
# BOOST
#===============================================================================
rosbuild_add_boost_directories()

#===============================================================================
# GENERATE BINARY
#===============================================================================

#Create Executable
rosbuild_add_executable( ${PROJECT_NAME} ${SOURCES} )

#Link against external libs
target_link_libraries( ${PROJECT_NAME} ${LIBS} )
rosbuild_link_boost( ${PROJECT_NAME} thread )
