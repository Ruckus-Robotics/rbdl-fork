CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT (rbdl)

SET ( RBDL_VERSION_MAJOR 2 )
SET ( RBDL_VERSION_MINOR 5 )
SET ( RBDL_VERSION_PATCH 0 )
SET ( RBDL_VERSION 
	${RBDL_VERSION_MAJOR}.${RBDL_VERSION_MINOR}.${RBDL_VERSION_PATCH}
)
SET ( RBDL_SO_VERSION 
	${RBDL_VERSION_MAJOR}.${RBDL_VERSION_MINOR}.${RBDL_VERSION_PATCH}
)
SET (PROJECT_VERSION ${RBDL_VERSION})

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake )

SET(INCLUDES_DIR ${PROJECT_SOURCE_DIR}/include)
# Source files for RBDL
SET ( RBDL_SOURCES 
	src/rbdl_version.cc
	src/rbdl_mathutils.cc
	src/rbdl_utils.cc
	src/Contacts.cc
	src/Dynamics.cc
	src/Logging.cc
	src/Joint.cc
	src/Model.cc
	src/Kinematics.cc
	)

include_directories( ${INCLUDES_DIR} ${EIGEN3_INCLUDE_DIR})

catkin_package(
 INCLUDE_DIRS ${INCLUDES_DIR} ${EIGEN3_INCLUDE_DIR}
 LIBRARIES ${PROJECT_NAME}
# CATKIN_DEPENDS
 DEPENDS Eigen3
)

# Options
OPTION (RBDL_BUILD_STATIC "Build statically linked library (otherwise dynamiclly linked)" OFF)
OPTION (RBDL_BUILD_TESTS "Build the test executables" OFF)
OPTION (RBDL_ENABLE_LOGGING "Enable logging (warning: major impact on performance!)" OFF)
OPTION (RBDL_USE_SIMPLE_MATH "Use slow math instead of the fast Eigen3 library (faster compilation)" OFF)
OPTION (RBDL_STORE_VERSION "Enable storing of version information in the library (requires build from valid repository)" OFF)
OPTION (RBDL_BUILD_ADDON_URDFREADER "Build the (experimental) urdf reader" OFF)
OPTION (RBDL_BUILD_ADDON_BENCHMARK "Build the benchmarking tool" OFF)
OPTION (RBDL_BUILD_ADDON_LUAMODEL "Build the lua model reader" OFF)
OPTION (RBDL_BUILD_PYTHON_WRAPPER "Build experimental python wrapper" OFF)

CONFIGURE_FILE (
	"${CMAKE_CURRENT_SOURCE_DIR}/include/rbdl/rbdl_config.h.cmake" 
	"${CMAKE_CURRENT_BINARY_DIR}/include/rbdl/rbdl_config.h"
	)