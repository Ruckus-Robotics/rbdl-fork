cmake_minimum_required(VERSION 2.8.3)
project(frl)

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

set(SOURCES_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDES_DIR ${PROJECT_SOURCE_DIR}/include)
set(TEST_DIR ${PROJECT_SOURCE_DIR}/test)

set(INCLUDES ${INCLUDES_DIR}/${PROJECT_NAME}/geometry/RigidBodyTransform.hpp
			 ${INCLUDES_DIR}/${PROJECT_NAME}/geometry/Point3.hpp
			 ${INCLUDES_DIR}/${PROJECT_NAME}/frames/ReferenceFrame.hpp
			 ${INCLUDES_DIR}/${PROJECT_NAME}/frames/FramePoint.hpp
			 ${INCLUDES_DIR}/${PROJECT_NAME}/frames/ReferenceFrameHolder.hpp
			 ${INCLUDES_DIR}/${PROJECT_NAME}/frames/FrameVector.hpp
             ${INCLUDES_DIR}/${PROJECT_NAME}/utils/Utilities.hpp)

include_directories(${INCLUDES_DIR} ${EIGEN3_INCLUDE_DIR})

## Declare a cpp library
 add_library(${PROJECT_NAME}
   ${INCLUDES}
 )

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

 catkin_package(
 INCLUDE_DIRS ${INCLUDES_DIR} ${EIGEN3_INCLUDE_DIR}
 LIBRARIES ${PROJECT_NAME}
# CATKIN_DEPENDS
 DEPENDS Eigen3
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

    catkin_add_gtest(RigidBodyTransform-test test/RigidBodyTransformTest.cpp)
    catkin_add_gtest(ReferenceFrame-test test/ReferenceFrameTest.cpp)
    catkin_add_gtest(Point3-test test/Point3Test.cpp)
    catkin_add_gtest(FramePoint-test test/FramePointTest.cpp)
    catkin_add_gtest(FrameVector-test test/FrameVectorTest.cpp)
#
    target_link_libraries(RigidBodyTransform-test ${PROJECT_NAME})
    target_link_libraries(ReferenceFrame-test ${PROJECT_NAME})
    target_link_libraries(Point3-test ${PROJECT_NAME})
    target_link_libraries(FramePoint-test ${PROJECT_NAME})
    target_link_libraries(FrameVector-test ${PROJECT_NAME})

endif()
