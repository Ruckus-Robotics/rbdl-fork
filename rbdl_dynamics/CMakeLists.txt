CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT (rbdl)

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

# INCLUDE(GNUInstallDirs)

# # Static / dynamic builds
# IF (RBDL_BUILD_STATIC)
#   ADD_LIBRARY ( rbdl-static STATIC ${RBDL_SOURCES} )
#   SET_TARGET_PROPERTIES ( rbdl-static PROPERTIES PREFIX "lib")
#   SET_TARGET_PROPERTIES ( rbdl-static PROPERTIES OUTPUT_NAME "rbdl")

# 	IF (RBDL_BUILD_ADDON_LUAMODEL)
# 		TARGET_LINK_LIBRARIES ( rbdl-static
# 			rbdl_luamodel-static
# 			)
# 	ENDIF (RBDL_BUILD_ADDON_LUAMODEL)

# 	INSTALL (TARGETS rbdl-static
# 	  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
# 		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
# 	)
# ELSE (RBDL_BUILD_STATIC)
# 	ADD_LIBRARY ( rbdl SHARED ${RBDL_SOURCES} )
# 	SET_TARGET_PROPERTIES ( rbdl PROPERTIES
# 		VERSION ${RBDL_VERSION}
# 		SOVERSION ${RBDL_SO_VERSION}
# 		)

# 	INSTALL (TARGETS rbdl
# 		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
# 		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
# 		)
# ENDIF (RBDL_BUILD_STATIC)

# IF (RBDL_STORE_VERSION)
# 	# Set versioning information that can be queried during runtime
# 	EXEC_PROGRAM("hg" ${CMAKE_CURRENT_SOURCE_DIR} ARGS "id -i"
# 		OUTPUT_VARIABLE RBDL_BUILD_REVISION)
# 	EXEC_PROGRAM("hg" ${CMAKE_CURRENT_SOURCE_DIR} ARGS "branch"
# 		OUTPUT_VARIABLE RBDL_BUILD_BRANCH)
# 	SET (RBDL_BUILD_TYPE ${CMAKE_BUILD_TYPE})
# ELSE (RBDL_STORE_VERSION)
# 	SET (RBDL_BUILD_REVISION "unknown")
# 	SET (RBDL_BUILD_BRANCH "unknown")
# 	SET (RBDL_BUILD_TYPE "unknown")
# ENDIF (RBDL_STORE_VERSION)

# CONFIGURE_FILE (
# 	"${CMAKE_CURRENT_SOURCE_DIR}/include/rbdl/rbdl_config.h.cmake" 
# 	"${CMAKE_CURRENT_BINARY_DIR}/include/rbdl/rbdl_config.h"
# 	)

# # Python wrapper
# IF (RBDL_BUILD_PYTHON_WRAPPER)
# 	add_subdirectory ( python )

# ENDIF (RBDL_BUILD_PYTHON_WRAPPER)

# # Installation
# FILE ( GLOB headers 
# 	${CMAKE_CURRENT_SOURCE_DIR}/include/rbdl/*.h
# 	${CMAKE_CURRENT_BINARY_DIR}/include/rbdl/rbdl_config.h
# 	)

# INSTALL ( FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rbdl )

# # Setup of SimpleMath install settings
# IF (RBDL_USE_SIMPLE_MATH)
# 	INSTALL ( DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/rbdl/SimpleMath"
# 		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rbdl
# 	)
# ENDIF (RBDL_USE_SIMPLE_MATH)

# # pkg-config
# CONFIGURE_FILE (
#   ${CMAKE_CURRENT_SOURCE_DIR}/rbdl.pc.cmake
#   ${CMAKE_CURRENT_BINARY_DIR}/rbdl.pc @ONLY
#   )
# INSTALL (
#   FILES  ${CMAKE_CURRENT_BINARY_DIR}/rbdl.pc
#   DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
#   )
