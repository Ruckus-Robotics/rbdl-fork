CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT (rbdl_urdfreader)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp urdf rbdl_dynamics)

include_directories( include ${catkin_INCLUDE_DIRS})

catkin_package(
 INCLUDE_DIRS ${INCLUDES_DIR} ${EIGEN3_INCLUDE_DIR}
 LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp urdf rbdl_dynamics
 # DEPENDS 
)

SET ( URDFREADER_SOURCES
	src/urdfreader.cc
	)

add_library(${PROJECT_NAME} ${URDFREADER_SOURCES})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_util src/rbdl_urdfreader_util.cc)

target_link_libraries(${PROJECT_NAME}_util ${PROJECT_NAME})