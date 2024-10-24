cmake_minimum_required(VERSION 3.18)
project(PDContacts)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)
#set(CMAKE_CXX_FLAGS "-ftime-trace -Wall")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -Wall -Wno-comment -O3")
set(CMAKE_C_FLAGS_RELEASE "-DNDEBUG -Wall -Wno-comment -O3")
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
      ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()
set(CMAKE_BUILD_RPATH_USE_ORIGIN on)
set(APPLICATION_USE_GUI on)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})


set(EIGEN_BUILD_PATH "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/eigen")

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(OpenMP REQUIRED)

if(BUILD_NIX_PACKAGE)
	set(BUILD_NIX_PACKAGE ON CACHE INTERNAL "")
else()
	set(BUILD_NIX_PACKAGE OFF CACHE INTERNAL "")
endif()

set(FCL_BUILD_PATH "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/fcl")
set(CCD_BUILD_PATH "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/libccd")
set(CNPY_BUILD_PATH "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/cnpy")

include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${PROJECT_SOURCE_DIR}/extern")
include_directories("${PROJECT_SOURCE_DIR}/extern/amgcl")
include_directories("${PROJECT_SOURCE_DIR}/extern/libigl/include")
include_directories("${FCL_BUILD_PATH}/include/")
include_directories("${CCD_BUILD_PATH}/include/")
add_subdirectory(extern)
add_subdirectory(src)
