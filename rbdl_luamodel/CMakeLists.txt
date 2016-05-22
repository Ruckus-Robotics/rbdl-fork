CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT (rbdl_luamodel)

find_package(catkin REQUIRED COMPONENTS rbdl_dynamics)
find_package(Lua51 REQUIRED)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(INCLUDES_DIR include)

catkin_package(
 INCLUDE_DIRS ${INCLUDES_DIR} ${LUA_INCLUDE_DIR}
 LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS rbdl_dynamics
 DEPENDS Lua51
)

include_directories( 
  ${INCLUDES_DIR}
  ${LUA_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  )

set ( LUAMODEL_SOURCES 
  src/luamodel.cc
  src/luatables.cc
  )

ADD_EXECUTABLE (rbdl_luamodel_util src/rbdl_luamodel_util.cc)

ADD_LIBRARY (${PROJECT_NAME} ${LUAMODEL_SOURCES} )

  TARGET_LINK_LIBRARIES (${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${LUA_LIBRARIES}
    )

  TARGET_LINK_LIBRARIES (rbdl_luamodel_util
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

  INSTALL (TARGETS ${PROJECT_NAME} rbdl_luamodel_util 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )