CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT (rbdl_dynamics)

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

find_package(catkin REQUIRED COMPONENTS frl)
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
CATKIN_DEPENDS frl
 DEPENDS Eigen3
)

include_directories( ${INCLUDES_DIR} ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

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

IF (RBDL_STORE_VERSION)
	EXEC_PROGRAM("git" $${PROJECT_SOURCE_DIR} ARGS "rev-parse --short HEAD"
		OUTPUT_VARIABLE RBDL_BUILD_REVISION)
	message(${OUTPUT_VARIABLE_RBDL_BUILD_REVISION})
	EXEC_PROGRAM("git" ${PROJECT_SOURCE_DIR} ARGS "branch"
		OUTPUT_VARIABLE RBDL_BUILD_BRANCH)
	SET (RBDL_BUILD_TYPE ${CMAKE_BUILD_TYPE})
ELSE (RBDL_STORE_VERSION)
	SET (RBDL_BUILD_REVISION "unknown")
	SET (RBDL_BUILD_BRANCH "unknown")
	SET (RBDL_BUILD_TYPE "unknown")
ENDIF (RBDL_STORE_VERSION)

CONFIGURE_FILE (
	"${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/rbdl_config.h.cmake" 
	"${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/rbdl_config.h"
	)

add_library(${PROJECT_NAME} ${RBDL_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS 
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*"
)

# Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

	catkin_add_gtest(BodyTests tests/BodyTests.cc)
	catkin_add_gtest(CalcAccelerationsTests tests/CalcAccelerationsTests.cc)
    catkin_add_gtest(CalcVelocitiesTests tests/CalcVelocitiesTests.cc)
    catkin_add_gtest(CompositeRigidBodyTests tests/CompositeRigidBodyTests.cc)
    catkin_add_gtest(ContactsTests tests/ContactsTests.cc)
    catkin_add_gtest(CustomJointMultiBodyTests tests/CustomJointMultiBodyTests.cc)
    catkin_add_gtest(CustomJointSingleBodyTests tests/CustomJointSingleBodyTests.cc)
    catkin_add_gtest(CustomJointTests tests/CustomJointTests.cc)
	catkin_add_gtest(DynamicsTests tests/DynamicsTests.cc)
    catkin_add_gtest(FloatingBaseTests tests/FloatingBaseTests.cc)
    catkin_add_gtest(ImpulsesTests tests/ImpulsesTests.cc)
    catkin_add_gtest(InverseDynamicsTests tests/InverseDynamicsTests.cc)
    catkin_add_gtest(KinematicsTests tests/KinematicsTests)
    catkin_add_gtest(MathTests tests/MathTests.cc)
    catkin_add_gtest(ModelTests tests/ModelTests.cc)
    catkin_add_gtest(MultiDofTests tests/MultiDofTests.cc)
    catkin_add_gtest(SparseFactorizationTests tests/SparseFactorizationTests.cc)
    catkin_add_gtest(SpatialAlgebraTests tests/SpatialAlgebraTests.cc)
    catkin_add_gtest(TwoLegModelTests tests/TwolegModelTests.cc)
    catkin_add_gtest(UtilsTests tests/UtilsTests.cc)
	#
	target_link_libraries(BodyTests ${PROJECT_NAME})
	target_link_libraries(CalcAccelerationsTests ${PROJECT_NAME})
    target_link_libraries(CalcVelocitiesTests ${PROJECT_NAME})
    target_link_libraries(CompositeRigidBodyTests ${PROJECT_NAME})
    target_link_libraries(ContactsTests ${PROJECT_NAME})
    target_link_libraries(CustomJointMultiBodyTests ${PROJECT_NAME})
    target_link_libraries(CustomJointSingleBodyTests ${PROJECT_NAME})
    target_link_libraries(CustomJointTests ${PROJECT_NAME})
	target_link_libraries(DynamicsTests ${PROJECT_NAME})
    target_link_libraries(FloatingBaseTests ${PROJECT_NAME})
    target_link_libraries(ImpulsesTests ${PROJECT_NAME})
    target_link_libraries(InverseDynamicsTests ${PROJECT_NAME})
    target_link_libraries(KinematicsTests ${PROJECT_NAME})
    target_link_libraries(MathTests ${PROJECT_NAME})
    target_link_libraries(ModelTests ${PROJECT_NAME})
    target_link_libraries(MultiDofTests ${PROJECT_NAME})
    target_link_libraries(SparseFactorizationTests ${PROJECT_NAME})
    target_link_libraries(SpatialAlgebraTests ${PROJECT_NAME})
    target_link_libraries(TwoLegModelTests ${PROJECT_NAME})
    target_link_libraries(UtilsTests ${PROJECT_NAME})

endif()